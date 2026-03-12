/**
 * Minimal C++ example: connect to Marvin robot and initialize dual LinkerHand (dexterous hand) for testing.
 *
 * Build from workspace (fa_w2_ws):
 *   colcon build --packages-select marvin_ros2_control
 *
 * Run (ensure robot is at 192.168.1.190 with hands on channel A/B):
 *   ros2 run marvin_ros2_control marvin_hand_init_example
 *   # or: ./install/marvin_ros2_control/lib/marvin_ros2_control/marvin_hand_init_example
 *
 * Requires: Marvin SDK, gripper_hardware_common, rclcpp.
 */

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <memory>
#include <unistd.h>
#include <cstdio>
#include <iomanip>
#include <thread>
#include <chrono>
#include <atomic>
#include <cstdint>
#include <mutex>

#include "MarvinSDK.h"
#include "rclcpp/rclcpp.hpp"
#include "marvin_ros2_control/tool/hands/linkerhand/dexterous_hand.h"
#include "marvin_ros2_control/tool/modbus_io.h"
#include "gripper_hardware_common/GripperBase.h"

using namespace marvin_ros2_control;

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
    bool ok = OnLinkTo(ip_parts[0], ip_parts[1], ip_parts[2], ip_parts[3]);
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

    DCSS test_frame_data;
    int frame_update = 0;
    for (int i = 0; i < 5; i++)
    {
        OnGetBuf(&test_frame_data);
        int current = test_frame_data.m_Out[0].m_OutFrameSerial;
        if (current != 0 && current != frame_update)
            frame_update = current;
        usleep(100000);
    }
    if (frame_update == 0)
        std::cerr << "Warning: no frame updates from robot. Connection may be invalid." << std::endl;
    else
        std::cout << "Robot connection verified (frame updates received)." << std::endl;
    return true;
}

/** Read hand status once: getStatus(), wait, read response, parse and print joint positions. */
static bool readAndPrintHandStatus(ModbusHand* hand, GetChDataFunc get_ch_data, const char* label)
{
    if (!hand || !get_ch_data) return false;
    hand->getStatus();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));

    unsigned char data_buf[256] = {0};
    long ch = COM1_CHANNEL;
    long received = get_ch_data(data_buf, &ch);
    if (received <= 0)
    {
        std::cerr << label << " read: no data received." << received << std::endl;
        return false;
    }

    std::vector<double> positions;
    if (!hand->processReadResponse(data_buf, static_cast<size_t>(received), positions))
    {
        std::cerr << label << " read: parse failed (" << received << " bytes)." << std::endl;
        return false;
    }

    std::cout << label << " status: " << positions.size() << " joints (raw " << received << " bytes):";
    for (size_t i = 0; i < positions.size(); ++i)
    {
        std::cout << " j" << i << "=" << std::fixed << std::setprecision(4) << positions[i];
        if (i < positions.size() - 1) std::cout << ",";
    }
    std::cout << std::endl;
    return true;
}

struct Async485Stats
{
    std::atomic<uint64_t> read_req_sent{0};
    std::atomic<uint64_t> rx_frames{0};
    std::atomic<uint64_t> rx_read_fc{0};
    std::atomic<uint64_t> rx_write_fc{0};  // kept for completeness, though we no longer send explicit write commands
    std::atomic<uint64_t> rx_other_fc{0};
    std::atomic<uint64_t> read_parse_ok{0};
    std::atomic<uint64_t> read_parse_fail{0};
};

struct Async485RawStats
{
    std::atomic<uint64_t> write_sent{0};
    std::atomic<uint64_t> rx_frames{0};
    std::atomic<uint64_t> rx_read_fc{0};
    std::atomic<uint64_t> rx_other_fc{0};
};

static void runAsync485ReadWriteTest(
    ModbusHand* hand,
    GetChDataFunc get_ch_data,
    Clear485Func clear_ch_data,
    int duration_ms,
    int write_period_ms,
    int read_period_ms)
{
    if (!hand || !get_ch_data || !clear_ch_data)
    {
        std::cerr << "Async 485 test: invalid function pointers/hand." << std::endl;
        return;
    }

    Async485Stats stats;
    std::atomic<bool> stop{false};
    std::atomic<bool> writer_done{false};
    const uint64_t max_frames = 100;
    std::vector<int> seq_state(max_frames, 0);  // 0=not sent, 1=sent, 2=paired
    std::mutex seq_mutex;

    // Try to start from a clean RX buffer
    clear_ch_data();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // clear_ch_data();

    const uint8_t slave_id = gripper_hardware_common::ModbusConfig::LinkerHand::RIGHT_HAND_SLAVE_ID;
    const uint8_t expected_read_fc = gripper_hardware_common::ModbusConfig::LinkerHand::READ_FUNCTION;  // typically 0x03

    // get-thread: only call get485 (OnGetChDataB) and parse/collect stats. No sending.
    std::thread reader([&]() {
        unsigned char data_buf[256] = {0};
        long ch = COM1_CHANNEL;

        while (!stop.load(std::memory_order_relaxed))
        {
            // Busy-poll get485; this thread must NOT call OnSetChData*.
            ch = COM1_CHANNEL;
            long received = get_ch_data(data_buf, &ch);
            if (received <= 0)
            {
                continue;
            }

            stats.rx_frames.fetch_add(1, std::memory_order_relaxed);

            // Classify the received frame by function code, to see read/write interleaving.
            if (received >= 2 && static_cast<uint8_t>(data_buf[0]) == slave_id)
            {
                const uint8_t fc = static_cast<uint8_t>(data_buf[1]);
                if (fc == expected_read_fc || (expected_read_fc == 0x03 && fc == 0x04) || (expected_read_fc == 0x04 && fc == 0x03))
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
                else if (fc == gripper_hardware_common::ModbusConfig::LinkerHand::WRITE_SINGLE_FUNCTION ||
                         fc == gripper_hardware_common::ModbusConfig::LinkerHand::WRITE_FUNCTION)
                {
                    stats.rx_write_fc.fetch_add(1, std::memory_order_relaxed);
                }
                else
                {
                    stats.rx_other_fc.fetch_add(1, std::memory_order_relaxed);
                }
            }

            std::vector<double> positions;
            if (hand->processReadResponse(data_buf, static_cast<size_t>(received), positions))
            {
                stats.read_parse_ok.fetch_add(1, std::memory_order_relaxed);
            }
            else
            {
                stats.read_parse_fail.fetch_add(1, std::memory_order_relaxed);
            }
        }
    });

    // Per requirement: start get-thread first, then start set-thread.
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // set-thread: do all sending (set485). Here we only send readStatus requests (no explicit write commands).
    // It sends a fixed number of frames (100), then marks writer_done=true.
    std::thread writer([&]() {
        for (uint64_t i = 0; i < max_frames && !stop.load(std::memory_order_relaxed); ++i)
        {
            // Send readStatus request (will produce read responses for get-thread to consume).
            stats.read_req_sent.fetch_add(1, std::memory_order_relaxed);
            {
                std::lock_guard<std::mutex> lk(seq_mutex);
                if (i < max_frames)
                    seq_state[static_cast<size_t>(i)] = 1;
            }
            (void)hand->getStatus();

            std::this_thread::sleep_for(std::chrono::milliseconds(write_period_ms));
        }
        writer_done.store(true, std::memory_order_relaxed);
    });

    // Wait until writer finishes sending its fixed number of frames
    const auto start = std::chrono::steady_clock::now();
    while (!writer_done.load(std::memory_order_relaxed))
    {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= duration_ms) break;  // safety timeout
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // After writer is done, keep reading for an extra 1s to drain remaining responses
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

    const uint64_t read_req = stats.read_req_sent.load(std::memory_order_relaxed);
    const uint64_t rx = stats.rx_frames.load(std::memory_order_relaxed);
    const uint64_t rx_read = stats.rx_read_fc.load(std::memory_order_relaxed);
    const uint64_t rx_write = stats.rx_write_fc.load(std::memory_order_relaxed);
    const uint64_t rx_other = stats.rx_other_fc.load(std::memory_order_relaxed);
    const uint64_t ok = stats.read_parse_ok.load(std::memory_order_relaxed);
    const uint64_t fail = stats.read_parse_fail.load(std::memory_order_relaxed);

    const double resp_rate = (read_req > 0)
        ? (100.0 * static_cast<double>(rx) / static_cast<double>(read_req))
        : 0.0;
    const double parse_ok_rate = (rx > 0)
        ? (100.0 * static_cast<double>(ok) / static_cast<double>(rx))
        : 0.0;

    std::vector<size_t> unmatched_indices;
    {
        std::lock_guard<std::mutex> lk(seq_mutex);
        for (size_t i = 0; i < max_frames; ++i)
        {
            if (seq_state[i] == 1)  // sent but never paired with a read_fc response
            {
                unmatched_indices.push_back(i);
            }
        }
    }

    std::cout << "\n=== Async 485 read/write test summary (channel B) ===\n";
    std::cout << "Duration(ms): " << duration_ms << "\n";
    std::cout << "Read requests (setStatus): sent=" << read_req << "\n";
    std::cout << "RX frames: total=" << rx
              << ", read_fc=" << rx_read
              << ", write_fc=" << rx_write
              << ", other_fc=" << rx_other << " (resp_rate=" << resp_rate << "%)\n";
    std::cout << "Read parse: ok=" << ok << ", fail=" << fail
              << " (ok_rate=" << parse_ok_rate << "%)\n";
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

// Pure raw Modbus async test on channel B, without using hand/gripper interfaces.
// Writer thread repeatedly sends fixed frame: 27 04 00 00 00 07 B6 CE.
// Reader thread busy-polls get485 and counts responses.
static void runAsyncRawFrameTest(
    GetChDataFunc get_ch_data,
    Clear485Func clear_ch_data,
    int duration_ms,
    int write_period_ms)
{
    if (!get_ch_data || !clear_ch_data)
    {
        std::cerr << "Async raw 485 test: invalid function pointers." << std::endl;
        return;
    }

    Async485RawStats stats;
    std::atomic<bool> stop{false};
    std::atomic<bool> writer_done{false};

    // Clear RX buffer
    clear_ch_data();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    clear_ch_data();

    const uint8_t slave_id = 0x27;  // right hand Modbus slave ID
    const uint8_t expected_read_fc = 0x04;
    unsigned char raw_frame[8] = {0x27, 0x04, 0x00, 0x00, 0x00, 0x07, 0xB6, 0xCE};

    std::thread reader([&]() {
        unsigned char buf[256] = {0};
        long ch = COM1_CHANNEL;
        while (!stop.load(std::memory_order_relaxed))
        {
            ch = COM1_CHANNEL;
            long received = get_ch_data(buf, &ch);
            if (received <= 0)
                continue;

            stats.rx_frames.fetch_add(1, std::memory_order_relaxed);

            if (received >= 2 && static_cast<uint8_t>(buf[0]) == slave_id)
            {
                const uint8_t fc = static_cast<uint8_t>(buf[1]);
                if (fc == expected_read_fc)
                {
                    stats.rx_read_fc.fetch_add(1, std::memory_order_relaxed);
                }
                else
                {
                    stats.rx_other_fc.fetch_add(1, std::memory_order_relaxed);
                }
            }
        }
    });

    std::thread writer([&]() {
        const uint64_t max_frames = 100;
        for (uint64_t i = 0; i < max_frames && !stop.load(std::memory_order_relaxed); ++i)
        {
            if (OnSetChDataB(raw_frame, 8, COM1_CHANNEL))
            {
                stats.write_sent.fetch_add(1, std::memory_order_relaxed);
            }
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

    std::cout << "\n=== Async RAW 485 test summary (channel B, frame 27 04 00 00 00 07 B6 CE) ===\n";
    std::cout << "Writes sent: " << write_sent << "\n";
    std::cout << "RX frames: total=" << rx
              << ", read_fc=" << rx_read
              << ", other_fc=" << rx_other
              << " (resp_rate=" << resp_rate << "%)\n";
    std::cout << "===============================================\n" << std::endl;
}

int main(int argc, char* argv[])
{
    const std::string robot_ip = "192.168.1.190";
    const int robot_port = 8080;

    rclcpp::init(argc, argv);

    if (!connectToRobot(robot_ip, robot_port))
    {
        rclcpp::shutdown();
        return 1;
    }

    // Dual LinkerHand O7: left = channel A (slave 0x28), right = channel B (slave 0x27)
    // NOTE: left hand disabled for now; only test right hand.
    // auto left_hand = std::make_unique<DexterousHandO7>(
    //     OnClearChDataA, OnSetChDataA, OnGetChDataA, true);
    auto right_hand = std::make_unique<DexterousHandO7>(
        OnClearChDataB, OnSetChDataB, OnGetChDataB, false);

    std::cout << "Initializing right hand (channel B, 7-DOF) ..." << std::endl;
    if (!static_cast<gripper_hardware_common::GripperBase*>(right_hand.get())->initialize())
    {
        std::cerr << "Right hand initialize failed." << std::endl;
        OnRelease();
        rclcpp::shutdown();
        return 1;
    }
    std::cout << "Right hand initialized." << std::endl;

    std::cout << "Dual LinkerHand O7 initialization complete." << std::endl;

    std::cout << "\nSelect test mode:\n"
              << "  1) Sync only  (single readStatus)\n"
              << "  2) Async only (set/get concurrent test)\n"
              << "  3) Both sync then async  [default]\n"
              << "  4) Async RAW Modbus (27 04 00 00 00 07 B6 CE)\n"
              << "Enter choice (1/2/3/4): ";
    int mode = 3;
    if (!(std::cin >> mode))
    {
        mode = 3;
        std::cin.clear();
    }

    if (mode == 1 || mode == 3)
    {
        std::cout << "Reading right hand (channel B) status (sync) ..." << std::endl;
        if (!readAndPrintHandStatus(right_hand.get(), OnGetChDataB, "Right"))
        {
            std::cerr << "Right hand status read failed." << std::endl;
        }
    }

    if (mode == 2 || mode == 3)
    {
        // Async 485 test: concurrent write (set485) + read (get485) on the same channel.
        // set-thread does all sends (including getStatus), get-thread only drains responses.
        std::cout << "\nRunning async 485 read/write test (channel B) ..." << std::endl;
        runAsync485ReadWriteTest(
            right_hand.get(),
            OnGetChDataB,
            OnClearChDataB,
            9000,  // duration_ms
            100,    // write_period_ms
            50     // read_period_ms
        );
    }

    if (mode == 4)
    {
        std::cout << "\nRunning async RAW 485 test (channel B, frame 27 04 00 00 00 07 B6 CE) ..." << std::endl;
        runAsyncRawFrameTest(
            OnGetChDataB,
            OnClearChDataB,
            5000,  // duration_ms
            50     // write_period_ms
        );
    }

    std::cout << "Done." << std::endl;
    OnRelease();
    rclcpp::shutdown();
    return 0;
}
