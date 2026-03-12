/**
 * Standalone async raw 485 test: no ROS, no marvin_ros2_control, no gripper/hand.
 * Only depends on Marvin SDK. Sends fixed Modbus frame 27 04 00 00 00 07 B6 CE on channel B,
 * one thread writes (set), one thread reads (get485) and counts responses.
 *
 * Build: colcon build --packages-select marvin_ros2_control
 * Run:   ./install/marvin_ros2_control/lib/marvin_ros2_control/async_raw_485_test_sdk_only
 *        (robot at 192.168.1.190, channel B / COM1)
 */

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic>
#include <cstdint>
#include <mutex>

#include "MarvinSDK.h"

namespace {

constexpr long COM1_CHANNEL = 2;

static bool connectToRobot(const std::string& ip, int port)
{
    std::vector<int> ip_parts;
    std::istringstream iss(ip);
    std::string token;
    while (std::getline(iss, token, '.'))
    {
        if (!token.empty())
            ip_parts.push_back(std::stoi(token));
    }
    if (ip_parts.size() != 4)
    {
        std::cerr << "Invalid IP: " << ip << std::endl;
        return false;
    }

    std::cout << "Connecting to Marvin at " << ip << ":" << port << " ..." << std::endl;
    bool ok = OnLinkTo(static_cast<unsigned char>(ip_parts[0]),
                       static_cast<unsigned char>(ip_parts[1]),
                       static_cast<unsigned char>(ip_parts[2]),
                       static_cast<unsigned char>(ip_parts[3]));
    if (!ok)
    {
        std::cerr << "OnLinkTo failed." << std::endl;
        return false;
    }

    usleep(100000);
    OnClearSet();
    OnClearErr_A();
    OnClearErr_B();
    OnSetSend();
    usleep(100000);

    std::cout << "Connected." << std::endl;
    return true;
}

struct AsyncRawStats
{
    std::atomic<uint64_t> write_sent{0};
    std::atomic<uint64_t> rx_frames{0};
    std::atomic<uint64_t> rx_read_fc{0};
    std::atomic<uint64_t> rx_other_fc{0};
};

static void runAsyncRawFrameTest(int duration_ms, int write_period_ms)
{
    AsyncRawStats stats;
    std::atomic<bool> stop{false};
    std::atomic<bool> writer_done{false};
    const uint64_t max_frames = 100;
    std::vector<int> seq_state(max_frames, 0);  // 0=not sent, 1=sent, 2=paired
    std::mutex seq_mutex;

    OnClearChDataB();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    OnClearChDataB();

    const uint8_t slave_id = 0x27;
    const uint8_t expected_read_fc = 0x04;
    unsigned char raw_frame[8] = {0x27, 0x04, 0x00, 0x00, 0x00, 0x07, 0xB6, 0xCE};

    std::thread reader([&]() {
        unsigned char buf[256] = {0};
        long ch = COM1_CHANNEL;
        while (!stop.load(std::memory_order_relaxed))
        {
            ch = COM1_CHANNEL;
            long received = OnGetChDataB(buf, &ch);
            if (received <= 0)
                continue;

            stats.rx_frames.fetch_add(1, std::memory_order_relaxed);

            if (received >= 2 && static_cast<uint8_t>(buf[0]) == slave_id)
            {
                const uint8_t fc = static_cast<uint8_t>(buf[1]);
                if (fc == expected_read_fc)
                {
                    stats.rx_read_fc.fetch_add(1, std::memory_order_relaxed);
                    // Pair this response with the earliest unpaired send sequence.
                    std::lock_guard<std::mutex> lk(seq_mutex);
                    for (size_t idx = 0; idx < max_frames; ++idx)
                    {
                        if (seq_state[idx] == 1)
                        {
                            seq_state[idx] = 2;
                            break;
                        }
                    }
                }
                else
                {
                    stats.rx_other_fc.fetch_add(1, std::memory_order_relaxed);
                }
            }
        }
    });

    // Ensure reader is running before writer starts sending frames
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::thread writer([&]() {
        for (uint64_t i = 0; i < max_frames && !stop.load(std::memory_order_relaxed); ++i)
        {
            {
                std::lock_guard<std::mutex> lk(seq_mutex);
                if (i < max_frames)
                    seq_state[static_cast<size_t>(i)] = 1;
            }
            if (OnSetChDataB(raw_frame, 8, COM1_CHANNEL))
                stats.write_sent.fetch_add(1, std::memory_order_relaxed);
            std::this_thread::sleep_for(std::chrono::milliseconds(write_period_ms));
        }
        writer_done.store(true, std::memory_order_relaxed);
    });

    const auto start = std::chrono::steady_clock::now();
    while (!writer_done.load(std::memory_order_relaxed))
    {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= duration_ms) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    const auto after_writer = std::chrono::steady_clock::now();
    while (true)
    {
        const auto elapsed_after = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - after_writer).count();
        if (elapsed_after >= 1000) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    stop.store(true, std::memory_order_relaxed);
    writer.join();
    reader.join();

    const uint64_t write_sent = stats.write_sent.load(std::memory_order_relaxed);
    const uint64_t rx = stats.rx_frames.load(std::memory_order_relaxed);
    const uint64_t rx_read = stats.rx_read_fc.load(std::memory_order_relaxed);
    const uint64_t rx_other = stats.rx_other_fc.load(std::memory_order_relaxed);

    const double resp_rate = (write_sent > 0)
        ? (100.0 * static_cast<double>(rx) / static_cast<double>(write_sent))
        : 0.0;

    std::vector<size_t> unmatched_indices;
    {
        std::lock_guard<std::mutex> lk(seq_mutex);
        for (size_t i = 0; i < max_frames; ++i)
        {
            if (seq_state[i] == 1) // sent but never paired with a read FC response
                unmatched_indices.push_back(i);
        }
    }

    std::cout << "\n=== Async RAW 485 test (SDK only, channel B, frame 27 04 00 00 00 07 B6 CE) ===\n";
    std::cout << "Writes sent: " << write_sent << "\n";
    std::cout << "RX frames: total=" << rx
              << ", read_fc=" << rx_read
              << ", other_fc=" << rx_other
              << " (resp_rate=" << resp_rate << "%)\n";
    std::cout << "Unpaired request indices (sent but no matching read FC response): ";
    if (unmatched_indices.empty())
    {
        std::cout << "none";
    }
    else
    {
        for (size_t idx : unmatched_indices)
        {
            std::cout << idx << " ";
        }
    }
    std::cout << "\n";
    std::cout << "===============================================\n" << std::endl;
}

} // namespace

int main(int argc, char* argv[])
{
    (void)argc;
    (void)argv;

    const std::string robot_ip = "192.168.1.190";
    const int robot_port = 8080;

    if (!connectToRobot(robot_ip, robot_port))
        return 1;

    std::cout << "Running async RAW 485 test (100 frames, then 1s drain) ..." << std::endl;
    runAsyncRawFrameTest(9000, 200);

    std::cout << "Done." << std::endl;
    OnRelease();
    return 0;
}
