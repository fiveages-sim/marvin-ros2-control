#pragma once

#include "marvin_ros2_control/tool/hands/modbus_hand.h"
#include "rclcpp/logging.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace marvin_ros2_control
{
    class FreedomHandV1 : public ModbusHand
    {
    public:
        static constexpr size_t JOINT_COUNT = 6;

        FreedomHandV1(Clear485Func clear_485, Send485Func send_485,
                      GetChDataFunc on_get_ch_data = nullptr,
                      bool is_left_hand = true,
                      long channel = COM1_CHANNEL)
            : ModbusHand(clear_485, send_485, on_get_ch_data, channel),
              slave_id_(is_left_hand ? kLeftSlaveId : kRightSlaveId)
        {
        }

    protected:
        static constexpr uint8_t kFrameHead = 0x5A;
        static constexpr uint8_t kFrameTail = 0x5D;
        static constexpr uint8_t kHostToDevice = 0x00;
        static constexpr uint8_t kDeviceToHost = 0x01;
        static constexpr uint8_t kMoveCommand = 0x10;
        static constexpr uint8_t kAngleQueryCommand = 0xF1;
        static constexpr uint8_t kMoveFrameLength = 0x13;
        static constexpr uint8_t kQueryFrameLength = 0x08;
        static constexpr uint8_t kAngleResponseLength = 0x0D;
        static constexpr uint8_t kLeftSlaveId = 0x00;
        static constexpr uint8_t kRightSlaveId = 0x01;
        static constexpr uint8_t kMaxProtocolAngle = 90;

        uint8_t slave_id_;
        std::array<double, JOINT_COUNT> lower_limits_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::array<double, JOINT_COUNT> upper_limits_{0.785, 0.29, 1.24, 1.24, 1.24, 1.24};

        bool initialize() override
        {
            RCLCPP_INFO(logger_, "Initializing Freedom-V1 hand (6-DOF, slave: 0x%02X)", slave_id_);
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
                RCLCPP_ERROR(logger_, "Freedom-V1: Invalid position size. Expected %zu, got %zu",
                             JOINT_COUNT, positions.size());
                return false;
            }

            std::array<uint8_t, JOINT_COUNT> angles{};
            for (size_t i = 0; i < JOINT_COUNT; ++i)
            {
                angles[i] = radiansToProtocolAngle(positions[i], i);
            }

            std::vector<uint8_t> frame = {
                kFrameHead,
                slave_id_,
                kMoveCommand,
                kMoveFrameLength,
                kHostToDevice,
                0x01,
                angles[0],
                0x01,
                angles[1],
                0x01,
                angles[2],
                0x01,
                angles[3],
                0x01,
                angles[4],
                0x01,
                angles[5],
                0x00,
                kFrameTail};
            frame[frame.size() - 2] = checksum(frame);
            if (channel_ == CAN_CHANNEL)
            {
                std::vector<uint8_t> frame1 = buildCanFrame(
                    makeCanId(slave_id_, kMoveCommand, 2, 1),
                    {0x01, angles[0], 0x01, angles[1], 0x01, angles[2], 0x01, angles[3]});
                std::vector<uint8_t> frame2 = buildCanFrame(
                    makeCanId(slave_id_, kMoveCommand, 2, 2),
                    {0x01, angles[4], 0x01, angles[5]});
                return sendRequest(frame1) && sendRequest(frame2);
            }
            return sendRequest(frame);
        }

        bool getStatus() override
        {
            if (channel_ == CAN_CHANNEL)
            {
                return sendRequest(buildCanFrame(
                    makeCanId(slave_id_, kAngleQueryCommand, 1, 1),
                    {0x00}));
            }

            std::vector<uint8_t> query = {
                kFrameHead,
                slave_id_,
                kAngleQueryCommand,
                kQueryFrameLength,
                kHostToDevice,
                0x00,
                0x00,
                kFrameTail};
            query[query.size() - 2] = checksum(query);
            return sendRequest(query);
        }

        bool processReadResponse(const uint8_t* data, size_t data_size,
                                 std::vector<double>& positions) override
        {
            positions.clear();
            if (channel_ == CAN_CHANNEL)
            {
                const uint8_t* payload = data;
                size_t payload_size = data_size;
                if (data_size >= 4)
                {
                    const uint32_t can_id =
                        static_cast<uint32_t>(data[0]) |
                        (static_cast<uint32_t>(data[1]) << 8) |
                        (static_cast<uint32_t>(data[2]) << 16) |
                        (static_cast<uint32_t>(data[3]) << 24);
                    if (canDeviceId(can_id) == slave_id_ &&
                        canCommand(can_id) == kAngleQueryCommand)
                    {
                        payload = data + 4;
                        payload_size = data_size - 4;
                    }
                }

                if (payload_size < JOINT_COUNT)
                {
                    return false;
                }
                positions.reserve(JOINT_COUNT);
                for (size_t i = 0; i < JOINT_COUNT; ++i)
                {
                    positions.push_back(protocolAngleToRadians(payload[i], i));
                }
                return true;
            }

            if (data_size != kAngleResponseLength ||
                data[0] != kFrameHead ||
                data[1] != slave_id_ ||
                data[2] != kAngleQueryCommand ||
                data[3] != kAngleResponseLength ||
                data[4] != kDeviceToHost ||
                data[data_size - 1] != kFrameTail ||
                checksum(std::vector<uint8_t>(data, data + data_size)) != data[data_size - 2])
            {
                return false;
            }

            positions.reserve(JOINT_COUNT);
            for (size_t i = 0; i < JOINT_COUNT; ++i)
            {
                positions.push_back(protocolAngleToRadians(data[5 + i], i));
            }
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
            RCLCPP_INFO(logger_, "Freedom-V1 hand deinitialized");
        }

        void resetState() override
        {
        }

    private:
        uint8_t radiansToProtocolAngle(double radians, size_t joint_index) const
        {
            const auto lower = lower_limits_[joint_index];
            const auto upper = upper_limits_[joint_index];
            if (upper <= lower)
            {
                return 0;
            }
            const auto normalized = std::clamp((radians - lower) / (upper - lower), 0.0, 1.0);
            const auto angle = std::lround(normalized * static_cast<double>(kMaxProtocolAngle));
            return static_cast<uint8_t>(std::clamp<long>(angle, 0, kMaxProtocolAngle));
        }

        double protocolAngleToRadians(uint8_t angle, size_t joint_index) const
        {
            const auto lower = lower_limits_[joint_index];
            const auto upper = upper_limits_[joint_index];
            if (upper <= lower)
            {
                return lower;
            }
            const auto normalized = std::clamp(
                static_cast<double>(angle) / static_cast<double>(kMaxProtocolAngle), 0.0, 1.0);
            return lower + normalized * (upper - lower);
        }

        static uint8_t checksum(const std::vector<uint8_t>& frame)
        {
            if (frame.size() < 4)
            {
                return 0;
            }
            uint16_t sum = 0;
            for (size_t i = 1; i + 2 < frame.size(); ++i)
            {
                sum += frame[i];
            }
            return static_cast<uint8_t>(sum & 0xFF);
        }

        static uint32_t makeCanId(uint8_t device_id, uint8_t command, uint8_t total_frames, uint8_t frame_seq)
        {
            return (static_cast<uint32_t>(device_id & 0x1F) << 24) |
                   (static_cast<uint32_t>(command) << 16) |
                   (static_cast<uint32_t>(total_frames) << 8) |
                   static_cast<uint32_t>(frame_seq);
        }

        static uint8_t canDeviceId(uint32_t can_id)
        {
            return static_cast<uint8_t>((can_id >> 24) & 0x1F);
        }

        static uint8_t canCommand(uint32_t can_id)
        {
            return static_cast<uint8_t>((can_id >> 16) & 0xFF);
        }

        static std::vector<uint8_t> buildCanFrame(uint32_t can_id, const std::vector<uint8_t>& payload)
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
    };
} // namespace marvin_ros2_control
