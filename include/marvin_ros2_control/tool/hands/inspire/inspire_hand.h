#pragma once

#include "marvin_ros2_control/tool/hands/modbus_hand.h"

#include "rclcpp/logging.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <string>
#include <vector>

namespace marvin_ros2_control
{
    class InspireHandRH56E2 : public ModbusHand
    {
    public:
        static constexpr size_t JOINT_COUNT = 6;

        InspireHandRH56E2(Clear485Func clear_485, Send485Func send_485,
                          GetChDataFunc on_get_ch_data = nullptr,
                          bool is_left_hand = true)
            : ModbusHand(clear_485, send_485, on_get_ch_data),
              slave_id_(is_left_hand ? kLeftHandSlaveId : kRightHandSlaveId),
              is_left_hand_(is_left_hand)
        {
        }

        bool initialize() override
        {
            RCLCPP_INFO(logger_, "Initializing Inspire RH56E2 (%s hand, slave: %u)",
                        is_left_hand_ ? "left" : "right", slave_id_);

            std::vector<uint16_t> modes(JOINT_COUNT, 0);
            if (!writeMultipleRegisters(slave_id_, kModeRegister, modes, kWriteMultipleRegisters))
            {
                RCLCPP_WARN(logger_, "Inspire RH56E2: failed to set mode registers for slave %u", slave_id_);
            }

            std::vector<uint16_t> speeds(JOINT_COUNT, kDefaultSpeed);
            if (!writeMultipleRegisters(slave_id_, kSpeedSetRegister, speeds, kWriteMultipleRegisters))
            {
                RCLCPP_WARN(logger_, "Inspire RH56E2: failed to set speed registers for slave %u", slave_id_);
            }

            std::vector<uint16_t> forces(JOINT_COUNT, kDefaultForce);
            if (!writeMultipleRegisters(slave_id_, kForceSetRegister, forces, kWriteMultipleRegisters))
            {
                RCLCPP_WARN(logger_, "Inspire RH56E2: failed to set force registers for slave %u", slave_id_);
            }

            return true;
        }

        bool move_gripper(int torque, int velocity, double position) override
        {
            std::vector<int> torques(JOINT_COUNT, torque);
            std::vector<int> velocities(JOINT_COUNT, velocity);
            std::vector<double> positions(JOINT_COUNT, position);
            return move_hand(torques, velocities, positions);
        }

        bool move_hand(
            const std::vector<int>& torques,
            const std::vector<int>& velocities,
            const std::vector<double>& positions) override
        {
            (void)torques;
            (void)velocities;

            if (positions.size() != JOINT_COUNT)
            {
                RCLCPP_ERROR(logger_, "Inspire RH56E2: expected %zu positions, got %zu",
                             JOINT_COUNT, positions.size());
                return false;
            }

            const std::vector<uint16_t> actuator_values = {
                jointToActuator(positions[5], 5),
                jointToActuator(positions[4], 4),
                jointToActuator(positions[3], 3),
                jointToActuator(positions[2], 2),
                jointToActuator(positions[1], 1),
                jointToActuator(positions[0], 0),
            };

            return writeMultipleRegisters(
                slave_id_, kAngleSetRegister, actuator_values, kWriteMultipleRegisters);
        }

        bool getStatus() override
        {
            return sendReadRequestAsync(
                slave_id_, kAngleActRegister, JOINT_COUNT, kReadHoldingRegisters);
        }

        bool processReadResponse(const uint8_t* data, size_t data_size,
                                 std::vector<double>& positions) override
        {
            positions.clear();

            const auto registers = parseModbusResponse(
                data, data_size, slave_id_, kReadHoldingRegisters);
            if (registers.size() < JOINT_COUNT)
            {
                return false;
            }

            positions = {
                actuatorToJoint(registers[5], 0),
                actuatorToJoint(registers[4], 1),
                actuatorToJoint(registers[3], 2),
                actuatorToJoint(registers[2], 3),
                actuatorToJoint(registers[1], 4),
                actuatorToJoint(registers[0], 5),
            };
            return true;
        }

        size_t getJointCount() const override
        {
            return JOINT_COUNT;
        }

        int mapJointNameToIndex(const std::string& joint_name) const override
        {
            std::string name_lower = joint_name;
            std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);

            if (name_lower.find("thumb_joint1") != std::string::npos) return 0;
            if (name_lower.find("thumb_joint2") != std::string::npos) return 1;
            if (name_lower.find("index_joint") != std::string::npos)  return 2;
            if (name_lower.find("middle_joint") != std::string::npos) return 3;
            if (name_lower.find("ring_joint") != std::string::npos)   return 4;
            if (name_lower.find("pinky_joint") != std::string::npos)  return 5;
            return -1;
        }

        void deinitialize() override
        {
            RCLCPP_INFO(logger_, "Inspire RH56E2 deinitialized");
        }

        void resetState() override {}

    private:
        static constexpr uint8_t kLeftHandSlaveId = 2;
        static constexpr uint8_t kRightHandSlaveId = 1;
        static constexpr uint8_t kReadHoldingRegisters = 0x03;
        static constexpr uint8_t kWriteMultipleRegisters = 0x10;

        static constexpr uint16_t kAngleSetRegister = 1040;
        static constexpr uint16_t kForceSetRegister = 1046;
        static constexpr uint16_t kSpeedSetRegister = 1052;
        static constexpr uint16_t kAngleActRegister = 1064;
        static constexpr uint16_t kModeRegister = 1100;

        static constexpr uint16_t kDefaultSpeed = 4000;
        static constexpr uint16_t kDefaultForce = 6000;

        static constexpr std::array<double, JOINT_COUNT> kLowerLimits{
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        static constexpr std::array<double, JOINT_COUNT> kUpperLimits{
            1.1641, 0.5864, 1.4381, 1.4381, 1.4381, 1.4381};
        static constexpr std::array<double, JOINT_COUNT> kRawOpen{
            1750.0, 1450.0, 1740.0, 1740.0, 1740.0, 1740.0};
        static constexpr std::array<double, JOINT_COUNT> kRawClosed{
            500.0, 1100.0, 900.0, 900.0, 900.0, 900.0};

        uint16_t jointToActuator(double radians, size_t joint_index) const
        {
            const auto lower = kLowerLimits[joint_index];
            const auto upper = kUpperLimits[joint_index];
            if (upper <= lower)
            {
                return static_cast<uint16_t>(std::lround(kRawOpen[joint_index]));
            }

            const auto ratio = std::clamp((radians - lower) / (upper - lower), 0.0, 1.0);
            const auto value =
                kRawOpen[joint_index] + ratio * (kRawClosed[joint_index] - kRawOpen[joint_index]);
            return static_cast<uint16_t>(std::clamp<long>(std::lround(value), 0, 2000));
        }

        double actuatorToJoint(uint16_t value, size_t joint_index) const
        {
            const auto lower = kLowerLimits[joint_index];
            const auto upper = kUpperLimits[joint_index];
            const auto raw_range = kRawClosed[joint_index] - kRawOpen[joint_index];
            if (upper <= lower || std::abs(raw_range) < 1e-9)
            {
                return lower;
            }

            const auto ratio =
                std::clamp((static_cast<double>(value) - kRawOpen[joint_index]) / raw_range, 0.0, 1.0);
            return lower + ratio * (upper - lower);
        }

        uint8_t slave_id_;
        bool is_left_hand_;
    };
} // namespace marvin_ros2_control
