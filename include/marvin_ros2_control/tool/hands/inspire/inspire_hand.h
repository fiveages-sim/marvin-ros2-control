#pragma once

#include "marvin_ros2_control/tool/hands/modbus_hand.h"
#include "rclcpp/logging.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace marvin_ros2_control
{
    class InspireHandE2 : public ModbusHand
    {
    public:
        static constexpr size_t JOINT_COUNT = 6;
        static constexpr size_t ACTUATOR_COUNT = 6;

        InspireHandE2(Clear485Func clear_485, Send485Func send_485,
                      GetChDataFunc on_get_ch_data = nullptr,
                      bool is_left_hand = true)
            : ModbusHand(clear_485, send_485, on_get_ch_data),
              slave_id_(is_left_hand ? kLeftSlaveId : kRightSlaveId)
        {
        }

    protected:
        static constexpr uint8_t kReadFunction = 0x03;
        static constexpr uint8_t kWriteFunction = 0x10;
        static constexpr uint8_t kLeftSlaveId = 0x02;
        static constexpr uint8_t kRightSlaveId = 0x01;
        static constexpr uint16_t kModeRegister = 1100;
        static constexpr uint16_t kAngleSetRegister = 1040;
        static constexpr uint16_t kForceSetRegister = 1046;
        static constexpr uint16_t kSpeedSetRegister = 1052;
        static constexpr uint16_t kAngleActRegister = 1064;
        static constexpr uint16_t kDefaultSpeed = 4000;
        static constexpr uint16_t kDefaultForce = 6000;

        uint8_t slave_id_;
        std::array<double, JOINT_COUNT> lower_limits_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::array<double, JOINT_COUNT> upper_limits_{1.1641, 0.5864, 1.4381, 1.4381, 1.4381, 1.4381};

        bool initialize() override
        {
            RCLCPP_INFO(logger_, "Initializing Inspire-E2 hand (6-DOF, slave: 0x%02X)", slave_id_);
            std::vector<uint16_t> modes(ACTUATOR_COUNT, 0);
            std::vector<uint16_t> speeds(ACTUATOR_COUNT, kDefaultSpeed);
            std::vector<uint16_t> forces(ACTUATOR_COUNT, kDefaultForce);

            if (!writeMultipleRegisters(slave_id_, kModeRegister, modes, kWriteFunction))
            {
                RCLCPP_WARN(logger_, "Inspire-E2: Failed to set mode register; continuing");
            }
            if (!writeMultipleRegisters(slave_id_, kSpeedSetRegister, speeds, kWriteFunction))
            {
                RCLCPP_WARN(logger_, "Inspire-E2: Failed to set speed register; continuing");
            }
            if (!writeMultipleRegisters(slave_id_, kForceSetRegister, forces, kWriteFunction))
            {
                RCLCPP_WARN(logger_, "Inspire-E2: Failed to set force register; continuing");
            }
            return true;
        }

        bool move_gripper(int torque, int velocity, double normalized_pos) override
        {
            (void)torque;
            (void)velocity;
            std::vector<int> torques(JOINT_COUNT, torque);
            std::vector<int> velocities(JOINT_COUNT, velocity);
            std::vector<double> positions(JOINT_COUNT, normalized_pos);
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
                RCLCPP_ERROR(logger_, "Inspire-E2: Invalid position size. Expected %zu, got %zu",
                             JOINT_COUNT, positions.size());
                return false;
            }

            std::array<double, JOINT_COUNT> joints{};
            std::copy_n(positions.begin(), JOINT_COUNT, joints.begin());
            const auto actuators = jointsToActuators(joints);
            return writeMultipleRegisters(
                slave_id_, kAngleSetRegister,
                std::vector<uint16_t>(actuators.begin(), actuators.end()),
                kWriteFunction);
        }

        bool getStatus() override
        {
            return sendReadRequestAsync(slave_id_, kAngleActRegister, ACTUATOR_COUNT, kReadFunction);
        }

        bool processReadResponse(const uint8_t* data, size_t data_size,
                                 std::vector<double>& positions) override
        {
            positions.clear();
            const auto registers = parseModbusResponse(data, data_size, slave_id_, kReadFunction);
            if (registers.size() < ACTUATOR_COUNT)
            {
                return false;
            }

            std::array<uint16_t, ACTUATOR_COUNT> actuators{};
            std::copy_n(registers.begin(), ACTUATOR_COUNT, actuators.begin());
            const auto joints = actuatorsToJoints(actuators);
            positions.assign(joints.begin(), joints.end());
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
            if (name_lower.find("index_joint") != std::string::npos) return 2;
            if (name_lower.find("middle_joint") != std::string::npos) return 3;
            if (name_lower.find("ring_joint") != std::string::npos) return 4;
            if (name_lower.find("pinky_joint") != std::string::npos) return 5;
            return -1;
        }

        void deinitialize() override
        {
            RCLCPP_INFO(logger_, "Inspire-E2 hand deinitialized");
        }

        void resetState() override
        {
        }

    private:
        std::array<uint16_t, ACTUATOR_COUNT> jointsToActuators(
            const std::array<double, JOINT_COUNT>& joints) const
        {
            return {
                jointToActuatorValue(joints[5], 5),
                jointToActuatorValue(joints[4], 4),
                jointToActuatorValue(joints[3], 3),
                jointToActuatorValue(joints[2], 2),
                jointToActuatorValue(joints[1], 1),
                jointToActuatorValue(joints[0], 0)};
        }

        std::array<double, JOINT_COUNT> actuatorsToJoints(
            const std::array<uint16_t, ACTUATOR_COUNT>& actuators) const
        {
            return {
                actuatorValueToJoint(actuators[5], 0),
                actuatorValueToJoint(actuators[4], 1),
                actuatorValueToJoint(actuators[3], 2),
                actuatorValueToJoint(actuators[2], 3),
                actuatorValueToJoint(actuators[1], 4),
                actuatorValueToJoint(actuators[0], 5)};
        }

        uint16_t jointToActuatorValue(double radians, size_t joint_index) const
        {
            constexpr std::array<double, JOINT_COUNT> raw_open{1750.0, 1450.0, 1740.0, 1740.0, 1740.0, 1740.0};
            constexpr std::array<double, JOINT_COUNT> raw_closed{500.0, 1100.0, 900.0, 900.0, 900.0, 900.0};

            const auto lower = lower_limits_[joint_index];
            const auto upper = upper_limits_[joint_index];
            if (upper <= lower)
            {
                return static_cast<uint16_t>(std::lround(raw_open[joint_index]));
            }

            const auto ratio = std::clamp((radians - lower) / (upper - lower), 0.0, 1.0);
            const auto value = raw_open[joint_index] + ratio * (raw_closed[joint_index] - raw_open[joint_index]);
            return static_cast<uint16_t>(std::clamp<long>(std::lround(value), 0, 2000));
        }

        double actuatorValueToJoint(uint16_t value, size_t joint_index) const
        {
            constexpr std::array<double, JOINT_COUNT> raw_open{1750.0, 1450.0, 1740.0, 1740.0, 1740.0, 1740.0};
            constexpr std::array<double, JOINT_COUNT> raw_closed{500.0, 1100.0, 900.0, 900.0, 900.0, 900.0};

            const auto lower = lower_limits_[joint_index];
            const auto upper = upper_limits_[joint_index];
            if (upper <= lower ||
                std::abs(raw_closed[joint_index] - raw_open[joint_index]) <
                    std::numeric_limits<double>::epsilon())
            {
                return lower;
            }

            const auto ratio = std::clamp(
                (static_cast<double>(value) - raw_open[joint_index]) /
                    (raw_closed[joint_index] - raw_open[joint_index]),
                0.0,
                1.0);
            return lower + ratio * (upper - lower);
        }
    };
} // namespace marvin_ros2_control
