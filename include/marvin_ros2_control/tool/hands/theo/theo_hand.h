#pragma once

#include "marvin_ros2_control/tool/hands/modbus_hand.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace marvin_ros2_control
{
    class TheoHandSTD16A : public ModbusHand
    {
    public:
        TheoHandSTD16A(Clear485Func clear_485, Send485Func send_485,
                       GetChDataFunc on_get_ch_data = nullptr,
                       long channel = COM1_CHANNEL);

        bool initialize() override;
        bool move_gripper(double normalized_torque, double normalized_velocity, double position) override;
        bool move_hand(const std::vector<double>& torques,
                       const std::vector<double>& velocities,
                       const std::vector<double>& positions) override;
        bool getStatus() override;
        size_t getJointCount() const override { return kJointCount; }
        int mapJointNameToIndex(const std::string& joint_name) const override;
        bool processReadResponse(const uint8_t* data, size_t data_size,
                                 std::vector<double>& positions) override;

    private:
        static uint16_t radiansToProtocol(double position, double lower, double upper);
        static double protocolToRadians(int raw_position, double lower, double upper);
        static std::string lower(const std::string& value);

        static constexpr uint8_t kSlaveId = 0x01;
        static constexpr uint8_t kReadFunction = 0x03;
        static constexpr uint8_t kWriteSingleFunction = 0x06;
        static constexpr uint8_t kWriteMultipleFunction = 0x10;
        static constexpr uint16_t kControlWordRegister = 0x0000;
        static constexpr uint16_t kTargetPositionRegister = 0x0001;
        static constexpr uint16_t kFeedbackPositionRegister = 0x0051;
        static constexpr uint16_t kEnableControlWord = 0x000F;
        static constexpr uint16_t kProtocolPositionMax = 9000;
        static constexpr size_t kJointCount = 16;
        static constexpr double kPi = 3.1415926535897931;

        static const std::array<const char*, kJointCount> kJointSuffixes;
        static const std::array<double, kJointCount> kLowerLimits;
        static const std::array<double, kJointCount> kUpperLimits;
    };
} // namespace marvin_ros2_control
