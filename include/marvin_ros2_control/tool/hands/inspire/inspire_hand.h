#pragma once

#include "marvin_ros2_control/tool/hands/modbus_hand.h"
#include "rclcpp/logging.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
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
            RCLCPP_INFO(logger_, "Initializing inspire_e2 hand (6-DOF, slave: 0x%02X)", slave_id_);
            std::vector<uint16_t> modes(ACTUATOR_COUNT, 0);
            std::vector<uint16_t> speeds(ACTUATOR_COUNT, kDefaultSpeed);
            std::vector<uint16_t> forces(ACTUATOR_COUNT, kDefaultForce);

            if (!writeMultipleRegisters(slave_id_, kModeRegister, modes, kWriteFunction))
            {
                RCLCPP_WARN(logger_, "inspire_e2: Failed to set mode register; continuing");
            }
            if (!writeMultipleRegisters(slave_id_, kSpeedSetRegister, speeds, kWriteFunction))
            {
                RCLCPP_WARN(logger_, "inspire_e2: Failed to set speed register; continuing");
            }
            if (!writeMultipleRegisters(slave_id_, kForceSetRegister, forces, kWriteFunction))
            {
                RCLCPP_WARN(logger_, "inspire_e2: Failed to set force register; continuing");
            }
            return true;
        }

        bool move_gripper(double normalized_torque, double normalized_velocity, double normalized_pos) override
        {
            std::vector<double> torques(JOINT_COUNT, std::clamp(normalized_torque, 0.0, 1.0));
            std::vector<double> velocities(JOINT_COUNT, std::clamp(normalized_velocity, 0.0, 1.0));
            std::vector<double> positions(JOINT_COUNT, normalized_pos);
            return move_hand(torques, velocities, positions);
        }

        bool move_hand(
            const std::vector<double>& torques,
            const std::vector<double>& velocities,
            const std::vector<double>& positions) override
        {
            (void)torques;
            (void)velocities;
            if (positions.size() != JOINT_COUNT)
            {
                RCLCPP_ERROR(logger_, "inspire_e2: Invalid position size. Expected %zu, got %zu",
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
            RCLCPP_INFO(logger_, "inspire_e2 hand deinitialized");
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

    class InspireHandE2Canfd : public ModbusHand
    {
    public:
        static constexpr size_t JOINT_COUNT = 6;
        static constexpr size_t ACTUATOR_COUNT = 6;

        InspireHandE2Canfd(Clear485Func clear_485, Send485Func send_485,
                           GetChDataFunc on_get_ch_data = nullptr,
                           bool is_left_hand = true,
                           long channel = CAN_CHANNEL)
            : ModbusHand(clear_485, send_485, on_get_ch_data, channel),
              hand_id_(is_left_hand ? kLeftHandId : kRightHandId)
        {
        }

        static constexpr uint32_t kAngleCommandId = 0x10C410FF;
        static constexpr uint32_t kAngleQueryId = 0x001422FF;
        static constexpr uint32_t kAngleFeedbackId = 0x04042202;
        static constexpr size_t kAnglePayloadBytes = ACTUATOR_COUNT * 2;
        static constexpr size_t kAngleFeedbackPayloadOffset = 16;

        static void encodeU16Le(uint16_t value, uint8_t* data)
        {
            data[0] = static_cast<uint8_t>(value & 0xFF);
            data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
        }

        static uint16_t decodeU16Le(const uint8_t* data)
        {
            return static_cast<uint16_t>(data[0]) |
                   static_cast<uint16_t>(static_cast<uint16_t>(data[1]) << 8);
        }

        static std::vector<uint8_t> buildCanfdFrame(uint32_t can_id, const std::vector<uint8_t>& payload)
        {
            std::vector<uint8_t> frame;
            frame.reserve(4 + payload.size());
            frame.push_back(static_cast<uint8_t>(can_id & 0xFF));
            frame.push_back(static_cast<uint8_t>((can_id >> 8) & 0xFF));
            frame.push_back(static_cast<uint8_t>((can_id >> 16) & 0xFF));
            frame.push_back(static_cast<uint8_t>((can_id >> 24) & 0xFF));
            frame.insert(frame.end(), payload.begin(), payload.end());
            return frame;
        }

        static std::string frameToHex(const std::vector<uint8_t>& frame)
        {
            return bytesToHex(frame.data(), frame.size());
        }

        static std::string bytesToHex(const uint8_t* data, size_t data_size)
        {
            if (data == nullptr)
            {
                return "";
            }
            std::string hex;
            hex.reserve(data_size * 3);
            char byte_buffer[4]{};
            for (size_t i = 0; i < data_size; ++i)
            {
                if (i != 0)
                {
                    hex.push_back(' ');
                }
                std::snprintf(byte_buffer, sizeof(byte_buffer), "%02X", data[i]);
                hex.append(byte_buffer);
            }
            return hex;
        }

        static std::vector<uint8_t> buildAngleCommandFrame(
            const std::array<uint16_t, ACTUATOR_COUNT>& protocol_angles)
        {
            std::vector<uint8_t> payload;
            payload.resize(kAnglePayloadBytes);
            for (size_t i = 0; i < ACTUATOR_COUNT; ++i)
            {
                encodeU16Le(protocol_angles[i], payload.data() + i * 2);
            }
            return buildCanfdFrame(kAngleCommandId, payload);
        }

        static std::vector<uint8_t> buildAngleQueryFrame()
        {
            return buildCanfdFrame(kAngleQueryId, {0x40});
        }

        static bool parseAngleFeedbackFrame(const uint8_t* data, size_t data_size,
                                            std::array<uint16_t, ACTUATOR_COUNT>& protocol_angles)
        {
            if (data == nullptr || data_size < 4)
            {
                return false;
            }

            const uint32_t can_id =
                static_cast<uint32_t>(data[0]) |
                (static_cast<uint32_t>(data[1]) << 8) |
                (static_cast<uint32_t>(data[2]) << 16) |
                (static_cast<uint32_t>(data[3]) << 24);

            size_t payload_offset = 0;
            if (can_id == kAngleFeedbackId)
            {
                payload_offset = kAngleFeedbackPayloadOffset;
            }
            else if (can_id == kAngleCommandId)
            {
                payload_offset = 4;
            }
            else
            {
                return false;
            }

            if (data_size < payload_offset + kAnglePayloadBytes)
            {
                return false;
            }

            for (size_t i = 0; i < ACTUATOR_COUNT; ++i)
            {
                protocol_angles[i] = decodeU16Le(data + payload_offset + i * 2);
            }
            return true;
        }

        static uint16_t jointRadiansToRaw(double radians, size_t joint_index)
        {
            const auto lower = kLowerLimits[joint_index];
            const auto upper = kUpperLimits[joint_index];
            if (upper <= lower)
            {
                return kRawOpen[joint_index];
            }

            const auto ratio = std::clamp((radians - lower) / (upper - lower), 0.0, 1.0);
            const auto value = static_cast<double>(kRawOpen[joint_index]) +
                               ratio * (static_cast<double>(kRawClosed[joint_index]) -
                                        static_cast<double>(kRawOpen[joint_index]));
            const auto min_value = std::min(kRawOpen[joint_index], kRawClosed[joint_index]);
            const auto max_value = std::max(kRawOpen[joint_index], kRawClosed[joint_index]);
            return static_cast<uint16_t>(
                std::clamp<long>(std::lround(value), min_value, max_value));
        }

        static double rawToJointRadians(uint16_t value, size_t joint_index)
        {
            const auto lower = kLowerLimits[joint_index];
            const auto upper = kUpperLimits[joint_index];
            if (upper <= lower ||
                kRawClosed[joint_index] == kRawOpen[joint_index])
            {
                return lower;
            }

            const auto ratio = std::clamp(
                (static_cast<double>(value) - static_cast<double>(kRawOpen[joint_index])) /
                    (static_cast<double>(kRawClosed[joint_index]) -
                     static_cast<double>(kRawOpen[joint_index])),
                0.0,
                1.0);
            return lower + ratio * (upper - lower);
        }

    protected:
        static constexpr uint8_t kLeftHandId = 0x02;
        static constexpr uint8_t kRightHandId = 0x01;
        static constexpr std::array<double, JOINT_COUNT> kLowerLimits{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        static constexpr std::array<double, JOINT_COUNT> kUpperLimits{1.1641, 0.5864, 1.4381, 1.4381, 1.4381, 1.4381};
        static constexpr std::array<uint16_t, JOINT_COUNT> kRawOpen{1700, 1450, 1720, 1720, 1720, 1720};
        static constexpr std::array<uint16_t, JOINT_COUNT> kRawClosed{500, 1200, 900, 900, 900, 900};

        uint8_t hand_id_;

        bool initialize() override
        {
            RCLCPP_INFO(logger_, "Initializing inspire_e2 CANFD hand (6-DOF, fixed-angle protocol, hand_id: 0x%02X)",
                        hand_id_);
            return true;
        }

        bool move_gripper(double normalized_torque, double normalized_velocity, double normalized_pos) override
        {
            std::vector<double> torques(JOINT_COUNT, std::clamp(normalized_torque, 0.0, 1.0));
            std::vector<double> velocities(JOINT_COUNT, std::clamp(normalized_velocity, 0.0, 1.0));
            std::vector<double> positions(JOINT_COUNT, normalized_pos);
            return move_hand(torques, velocities, positions);
        }

        bool move_hand(
            const std::vector<double>& torques,
            const std::vector<double>& velocities,
            const std::vector<double>& positions) override
        {
            (void)torques;
            (void)velocities;
            if (positions.size() != JOINT_COUNT)
            {
                RCLCPP_ERROR(logger_, "inspire_e2_canfd: Invalid position size. Expected %zu, got %zu",
                             JOINT_COUNT, positions.size());
                return false;
            }

            std::array<double, JOINT_COUNT> joints{};
            std::copy_n(positions.begin(), JOINT_COUNT, joints.begin());
            //(void)joints;
            // Temporarily disabled for CANFD feedback/RViz debugging.
            return sendCanfdFrame("angle_command", buildAngleCommandFrame(jointsToProtocolAngles(joints)));
           // return true;
        }

        bool getStatus() override
        {
            return sendCanfdFrame("angle_query", buildAngleQueryFrame());
        }

        bool processReadResponse(const uint8_t* data, size_t data_size,
                                 std::vector<double>& positions) override
        {
            positions.clear();
            const size_t log_size = std::min(data_size, static_cast<size_t>(64));
            const auto rx_hex = bytesToHex(data, log_size);
            RCLCPP_INFO(logger_, "inspire_e2_canfd rx raw (%zu bytes, showing %zu): %s",
                        data_size, log_size, rx_hex.c_str());
            std::array<uint16_t, ACTUATOR_COUNT> protocol_angles{};
            if (!parseAngleFeedbackFrame(data, data_size, protocol_angles))
            {
                return false;
            }

            const auto joints = protocolAnglesToJoints(protocol_angles);
            positions.assign(joints.begin(), joints.end());
            RCLCPP_INFO(
                logger_,
                "inspire_e2_canfd rx angles raw[pinky=%u ring=%u middle=%u index=%u thumb_bend=%u thumb_rotate=%u] "
                "rad[thumb_rotate=%.4f thumb_bend=%.4f index=%.4f middle=%.4f ring=%.4f pinky=%.4f]",
                protocol_angles[0], protocol_angles[1], protocol_angles[2],
                protocol_angles[3], protocol_angles[4], protocol_angles[5],
                joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
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
            RCLCPP_INFO(logger_, "inspire_e2 CANFD hand deinitialized");
        }

        void resetState() override
        {
        }

    private:
        bool sendCanfdFrame(const char* label, const std::vector<uint8_t>& frame)
        {
            const auto hex = frameToHex(frame);
            RCLCPP_INFO(logger_, "inspire_e2_canfd tx %s: %s", label, hex.c_str());
            return sendRequest(frame);
        }

        std::array<uint16_t, ACTUATOR_COUNT> jointsToProtocolAngles(
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

        std::array<double, JOINT_COUNT> protocolAnglesToJoints(
            const std::array<uint16_t, ACTUATOR_COUNT>& protocol_angles) const
        {
            return {
                actuatorValueToJoint(protocol_angles[5], 0),
                actuatorValueToJoint(protocol_angles[4], 1),
                actuatorValueToJoint(protocol_angles[3], 2),
                actuatorValueToJoint(protocol_angles[2], 3),
                actuatorValueToJoint(protocol_angles[1], 4),
                actuatorValueToJoint(protocol_angles[0], 5)};
        }

        uint16_t jointToActuatorValue(double radians, size_t joint_index) const
        {
            return jointRadiansToRaw(radians, joint_index);
        }

        double actuatorValueToJoint(uint16_t value, size_t joint_index) const
        {
            return rawToJointRadians(value, joint_index);
        }
    };
} // namespace marvin_ros2_control
