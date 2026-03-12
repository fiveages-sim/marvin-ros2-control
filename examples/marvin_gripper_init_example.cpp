/**
 * Minimal C++ example: connect to Marvin robot and initialize dual Changingtek 90D gripper.
 *
 * Build from workspace (fa_w2_ws):
 *   colcon build --packages-select marvin_ros2_control --cmake-args -DMARVIN_BUILD_EXAMPLE=ON
 *
 * Run (ensure robot is at 192.168.1.190):
 *   ros2 run marvin_ros2_control marvin_gripper_init_example
 *   # or: ./install/marvin_ros2_control/lib/marvin_ros2_control/marvin_gripper_init_example
 *
 * Requires: Marvin SDK, gripper_hardware_common, rclcpp.
 */

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <memory>
#include <cstring>
#include <unistd.h>

#include "MarvinSDK.h"
#include "rclcpp/rclcpp.hpp"
#include "marvin_ros2_control/tool/grippers/changingtek/changingtek_gripper.h"
#include "marvin_ros2_control/tool/modbus_io.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
#include <cstdio>
#include <iomanip>
#include <thread>
#include <chrono>
#include <atomic>
#include <cstdint>

using namespace marvin_ros2_control;

/** Single read: send getStatus(), read response. Returns true if response received and response function code matches send. */
static bool doOneRead(marvin_ros2_control::ModbusGripper* gripper, GetChDataFunc get_ch_data,
                      uint8_t send_function_code, uint8_t* out_response_fc, long* out_received,
                      uint8_t* out_buf = nullptr, size_t out_buf_size = 0)
{
    if (!gripper || !get_ch_data) return false;
    gripper->getStatus();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    unsigned char data_buf[256] = {0};
    long ch = COM1_CHANNEL;
    long received = get_ch_data(data_buf, &ch);
    if (out_received) *out_received = received;
    if (received > 0 && out_buf && out_buf_size > 0)
    {
        size_t copy_len = static_cast<size_t>(received) < out_buf_size ? static_cast<size_t>(received) : out_buf_size;
        std::memcpy(out_buf, data_buf, copy_len);
    }
    if (received <= 0)
    {
        return false;
    }
    if (out_response_fc && received >= 2)
        *out_response_fc = data_buf[1];
    return (received >= 2 && data_buf[1] == send_function_code);
}

/** Run 100 reads, count how many times response function code equals send function code, and log the result. */
static void runRead100AndLogFcMatch(marvin_ros2_control::ModbusGripper* gripper, GetChDataFunc get_ch_data,
                                    const char* label)
{
    const uint8_t send_fc = gripper_hardware_common::ModbusConfig::Changingtek90D::READ_FUNCTION;
    constexpr int expected_size = 9;
    const int num_reads = 100;
    int match_count = 0;
    int same_fc_not_size_9_count = 0;

    unsigned char read_buf[256] = {0};

    for (int i = 0; i < num_reads; ++i)
    {
        uint8_t response_fc = 0;
        long received = 0;
        bool match = doOneRead(gripper, get_ch_data, send_fc, &response_fc, &received, read_buf, sizeof(read_buf));
        if (match)
        {
            match_count++;
            if (received != expected_size)
                same_fc_not_size_9_count++;
        }
        else if (received >= 2 && response_fc != send_fc)
        {
            std::cout << "  " << label << " non-matching fc (expected 0x" << std::hex << static_cast<int>(send_fc)
                      << ", got 0x" << static_cast<int>(response_fc) << std::dec << ", " << received
                      << " bytes):";
            for (long k = 0; k < received && k < 256; ++k)
                std::cout << " " << std::setw(2) << std::setfill('0') << std::hex << static_cast<int>(read_buf[k]);
            std::cout << std::dec << std::setfill(' ');
            std::cout << std::endl;
            std::cout << "  Pausing 10 s after non-matching fc." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    std::cout << label << ": " << match_count << "/" << num_reads
              << " responses had function code 0x" << std::hex << static_cast<int>(send_fc) << std::dec
              << " (same as send)." << std::endl;
    std::cout << "  " << label << ": " << same_fc_not_size_9_count
              << " of those had size != " << expected_size << " bytes." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
}

/** Read current tool status once: send getStatus(), wait, read response, parse and print. */
static bool readAndPrintToolStatus(
    marvin_ros2_control::ModbusGripper* gripper,
    GetChDataFunc get_ch_data,
    bool (*clear_ch)(),
    const char* label)
{
    if (!gripper || !get_ch_data) return false;
    // clear_ch();
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    gripper->getStatus();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    unsigned char data_buf[256] = {0};
    long ch = COM1_CHANNEL;
    long received = get_ch_data(data_buf, &ch);
    if (received <= 0)
    {
        std::cerr << label << " read: no data received." << std::endl;
        return false;
    }

    int torque = 0, velocity = 0;
    double position = 0.0;
    if (!gripper->processReadResponse(data_buf, static_cast<size_t>(received), torque, velocity, position))
    {
        std::cerr << label << " read: parse failed." << std::endl;
        return false;
    }

    std::cout << label << " status: position=" << position << " velocity=" << velocity
              << " torque=" << torque << " (raw " << received << " bytes)" << std::endl;
    return true;
}

struct Async485GripperStats
{
    std::atomic<uint64_t> read_req_sent{0};
    std::atomic<uint64_t> rx_frames{0};
    std::atomic<uint64_t> rx_read_fc{0};
    std::atomic<uint64_t> rx_write_fc{0};
    std::atomic<uint64_t> rx_other_fc{0};
    std::atomic<uint64_t> read_parse_ok{0};
    std::atomic<uint64_t> read_parse_fail{0};
};

static void runAsync485ReadWriteTestGripper(
    ModbusGripper* gripper,
    GetChDataFunc get_ch_data,
    Clear485Func clear_ch_data,
    int duration_ms,
    int write_period_ms)
{
    if (!gripper || !get_ch_data || !clear_ch_data)
    {
        std::cerr << "Async 485 gripper test: invalid function pointers/gripper." << std::endl;
        return;
    }

    Async485GripperStats stats;
    std::atomic<bool> stop{false};
    std::atomic<bool> writer_done{false};

    // Start from a clean RX buffer
    clear_ch_data();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    // clear_ch_data();

    const uint8_t slave_id = gripper_hardware_common::ModbusConfig::Changingtek90D::SLAVE_ID;
    const uint8_t expected_read_fc = gripper_hardware_common::ModbusConfig::Changingtek90D::READ_FUNCTION;

    // get-thread: only call get485 (OnGetChData*) and parse/collect stats. No sending.
    std::thread reader([&]() {
        unsigned char data_buf[256] = {0};
        long ch = COM1_CHANNEL;

        while (!stop.load(std::memory_order_relaxed))
        {
            ch = COM1_CHANNEL;
            long received = get_ch_data(data_buf, &ch);
            if (received <= 0)
            {
                continue;
            }

            stats.rx_frames.fetch_add(1, std::memory_order_relaxed);

            // Classify by function code
            if (received >= 2 && static_cast<uint8_t>(data_buf[0]) == slave_id)
            {
                const uint8_t fc = static_cast<uint8_t>(data_buf[1]);
                if (fc == expected_read_fc || (expected_read_fc == 0x03 && fc == 0x04) || (expected_read_fc == 0x04 && fc == 0x03))
                {
                    stats.rx_read_fc.fetch_add(1, std::memory_order_relaxed);
                }
                else if (fc == gripper_hardware_common::ModbusConfig::Changingtek90D::WRITE_SINGLE_FUNCTION ||
                         fc == gripper_hardware_common::ModbusConfig::Changingtek90D::WRITE_FUNCTION)
                {
                    stats.rx_write_fc.fetch_add(1, std::memory_order_relaxed);
                }
                else
                {
                    stats.rx_other_fc.fetch_add(1, std::memory_order_relaxed);
                }
            }

            int torque = 0;
            int velocity = 0;
            double position = 0.0;
            if (gripper->processReadResponse(data_buf, static_cast<size_t>(received), torque, velocity, position))
            {
                stats.read_parse_ok.fetch_add(1, std::memory_order_relaxed);
            }
            else
            {
                stats.read_parse_fail.fetch_add(1, std::memory_order_relaxed);
            }
        }
    });

    // set-thread: only send readStatus requests; fixed 100 frames.
    std::thread writer([&]() {
        const uint64_t max_frames = 100;
        for (uint64_t i = 0; i < max_frames && !stop.load(std::memory_order_relaxed); ++i)
        {
            stats.read_req_sent.fetch_add(1, std::memory_order_relaxed);
            (void)gripper->getStatus();
            std::this_thread::sleep_for(std::chrono::milliseconds(write_period_ms));
        }
        writer_done.store(true, std::memory_order_relaxed);
    });

    // Wait until writer finishes sending its fixed number of frames (or duration timeout)
    const auto start = std::chrono::steady_clock::now();
    while (!writer_done.load(std::memory_order_relaxed))
    {
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (elapsed >= duration_ms) break;
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

    std::cout << "\n=== Async 485 read/write test summary (Changingtek 90D, channel B) ===\n";
    std::cout << "Read requests (getStatus): sent=" << read_req << "\n";
    std::cout << "RX frames: total=" << rx
              << ", read_fc=" << rx_read
              << ", write_fc=" << rx_write
              << ", other_fc=" << rx_other
              << " (resp_rate=" << resp_rate << "%)\n";
    std::cout << "Read parse: ok=" << ok << ", fail=" << fail
              << " (ok_rate=" << parse_ok_rate << "%)\n";
    std::cout << "===============================================\n" << std::endl;
}

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

    // Optional: verify with frame updates
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
    {
        std::cerr << "Warning: no frame updates from robot. Connection may be invalid." << std::endl;
    }
    else
    {
        std::cout << "Robot connection verified (frame updates received)." << std::endl;
    }
    return true;
}

int main(int argc, char* argv[])
{
    const std::string robot_ip = "192.168.1.190";
    const int robot_port = 8080;

    // Required for ModbusIO static logger used by gripper code
    rclcpp::init(argc, argv);

    if (!connectToRobot(robot_ip, robot_port))
    {
        rclcpp::shutdown();
        return 1;
    }

    // Dual Changingtek 90D: left = channel A, right = channel B
    auto left_gripper = std::make_unique<ChangingtekGripper90D>(
        OnClearChDataA, OnSetChDataA, OnGetChDataA);
    auto right_gripper = std::make_unique<ChangingtekGripper90D>(
        OnClearChDataB, OnSetChDataB, OnGetChDataB);
    // OnClearChDataB();
    // OnClearChDataA();
   
    std::cout << "Initializing left gripper (channel A) ..." << std::endl;
    if (!left_gripper->initialize())
    {
        std::cerr << "Left gripper initialize failed." << std::endl;
        OnRelease();
        rclcpp::shutdown();
        return 1;
    }
    std::cout << "Left gripper initialized." << std::endl;

    std::cout << "Initializing right gripper (channel B) ..." << std::endl;
    if (!right_gripper->initialize())
    {
        std::cerr << "Right gripper initialize failed." << std::endl;
        OnRelease();
        rclcpp::shutdown();
        return 1;
    }
    std::cout << "Right gripper initialized." << std::endl;

    std::cout << "Dual Changingtek 90D gripper initialization complete." << std::endl;

    std::cout << "\nSelect test mode:\n"
              << "  1) Sync only  (100 reads per gripper + single status)\n"
              << "  2) Async only (set/get concurrent test on right gripper)\n"
              << "  3) Both sync then async  [default]\n"
              << "Enter choice (1/2/3): ";
    int mode = 3;
    if (!(std::cin >> mode))
    {
        mode = 3;
        std::cin.clear();
    }

    if (mode == 1 || mode == 3)
    {
        // Run 100 reads per gripper and log how many responses had function code matching send (0x03)
        std::cout << "Running 100 reads per gripper, checking response function code vs send..." << std::endl;
        runRead100AndLogFcMatch(left_gripper.get(), OnGetChDataA, "Left (ch A)");
        runRead100AndLogFcMatch(right_gripper.get(), OnGetChDataB, "Right (ch B)");

        // Read current status from both tools once and print
        std::cout << "Reading left gripper (channel A) status ..." << std::endl;
        if (!readAndPrintToolStatus(left_gripper.get(), OnGetChDataA, OnClearChDataA, "Left"))
        {
            std::cerr << "Left gripper status read failed." << std::endl;
        }

        std::cout << "Reading right gripper (channel B) status ..." << std::endl;
        if (!readAndPrintToolStatus(right_gripper.get(), OnGetChDataB, OnClearChDataB, "Right"))
        {
            std::cerr << "Right gripper status read failed." << std::endl;
        }
    }

    if (mode == 2 || mode == 3)
    {
        std::cout << "\nRunning async 485 read/write test on right gripper (channel B) ..." << std::endl;
        runAsync485ReadWriteTestGripper(
            right_gripper.get(),
            OnGetChDataB,
            OnClearChDataB,
            5000,  // safety timeout ms
            30     // write_period_ms between getStatus sends
        );
    }

    std::cout << "Done." << std::endl;
    OnRelease();
    rclcpp::shutdown();
    return 0;
}
