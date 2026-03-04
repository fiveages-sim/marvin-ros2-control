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
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    unsigned char data_buf[256] = {0};
    long ch = COM1_CHANNEL;
    long received = get_ch_data(data_buf, &ch);
    if (received <= 0)
    {
        std::cerr << label << " read: no data received." << std::endl;
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
    auto left_hand = std::make_unique<DexterousHandO7>(
        OnClearChDataA, OnSetChDataA, OnGetChDataA, true);
    auto right_hand = std::make_unique<DexterousHandO7>(
        OnClearChDataB, OnSetChDataB, OnGetChDataB, false);

    std::cout << "Initializing left hand (channel A, 7-DOF) ..." << std::endl;
    if (!static_cast<gripper_hardware_common::GripperBase*>(left_hand.get())->initialize())
    {
        std::cerr << "Left hand initialize failed." << std::endl;
        OnRelease();
        rclcpp::shutdown();
        return 1;
    }
    std::cout << "Left hand initialized." << std::endl;

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

    std::cout << "Reading left hand (channel A) status ..." << std::endl;
    if (!readAndPrintHandStatus(left_hand.get(), OnGetChDataA, "Left"))
        std::cerr << "Left hand status read failed." << std::endl;

    std::cout << "Reading right hand (channel B) status ..." << std::endl;
    if (!readAndPrintHandStatus(right_hand.get(), OnGetChDataB, "Right"))
        std::cerr << "Right hand status read failed." << std::endl;

    std::cout << "Done." << std::endl;
    OnRelease();
    rclcpp::shutdown();
    return 0;
}
