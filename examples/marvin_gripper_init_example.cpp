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

    std::cout << "Done." << std::endl;
    OnRelease();
    rclcpp::shutdown();
    return 0;
}
