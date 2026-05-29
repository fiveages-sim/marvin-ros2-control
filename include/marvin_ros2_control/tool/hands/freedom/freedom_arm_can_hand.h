#pragma once

#include "marvin_ros2_control/tool/hands/freedom/freedom_arm_can_protocol.h"
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
    class FreedomArmCanHand : public ModbusHand
    {
    public:
        static constexpr std::size_t JOINT_COUNT = FreedomArmCanProtocol::kJointCount;

        FreedomArmCanHand(Clear485Func clear_485, Send485Func send_485,
                          GetChDataFunc on_get_ch_data = nullptr,
                          bool is_left_hand = true)
            : ModbusHand(clear_485, send_485, on_get_ch_data),
              device_id_(is_left_hand ? kLeftHandDeviceId : kRightHandDeviceId),
              is_left_hand_(is_left_hand)
        {
        }

        bool initialize() override
        {
            RCLCPP_INFO(logger_, "Initializing Freedom arm CAN hand (%s hand, device_id: %u)",
                        is_left_hand_ ? "left" : "right", device_id_);
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
                RCLCPP_ERROR(logger_, "Freedom arm CAN hand: expected %zu positions, got %zu",
                             JOINT_COUNT, positions.size());
                return false;
            }

            std::array<uint8_t, JOINT_COUNT> angles{};
            for (std::size_t i = 0; i < JOINT_COUNT; ++i)
            {
                angles[i] = radiansToProtocolAngle(positions[i], i);
            }

            for (const auto& frame : FreedomArmCanProtocol::buildMoveFrames(device_id_, angles))
            {
                if (!sendFrame(frame))
                {
                    return false;
                }
            }
            return true;
        }

        bool getStatus() override
        {
            return sendFrame(FreedomArmCanProtocol::buildAngleQueryFrame(device_id_));
        }

        bool processReadResponse(const uint8_t* data, size_t data_size,
                                 std::vector<double>& positions) override
        {
            std::array<uint8_t, JOINT_COUNT> angles{};
            if (!FreedomArmCanProtocol::parseFeedbackFrame(device_id_, data, data_size, angles))
            {
                return false;
            }

            positions.clear();
            positions.reserve(JOINT_COUNT);
            for (std::size_t i = 0; i < JOINT_COUNT; ++i)
            {
                positions.push_back(protocolAngleToRadians(angles[i], i));
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
            if (name_lower.find("index_joint") != std::string::npos)  return 2;
            if (name_lower.find("middle_joint") != std::string::npos) return 3;
            if (name_lower.find("ring_joint") != std::string::npos)   return 4;
            if (name_lower.find("pinky_joint") != std::string::npos)  return 5;
            return -1;
        }

        long communicationChannel() const override
        {
            return CANFD_CHANNEL;
        }

        void deinitialize() override
        {
            RCLCPP_INFO(logger_, "Freedom arm CAN hand deinitialized");
        }

        void resetState() override {}

    private:
        static constexpr uint8_t kLeftHandDeviceId = 0;
        static constexpr uint8_t kRightHandDeviceId = 1;

        static constexpr std::array<double, JOINT_COUNT> kLowerLimits{
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        static constexpr std::array<double, JOINT_COUNT> kUpperLimits{
            0.785, 0.29, 1.24, 1.24, 1.24, 1.24};

        bool sendFrame(const std::vector<uint8_t>& frame)
        {
            if (frame.empty() || frame.size() > MAX_BUFFER_SIZE)
            {
                return false;
            }
            std::vector<uint8_t> tx_frame = frame;
            return send_485_(
                tx_frame.data(),
                static_cast<long>(tx_frame.size()),
                CANFD_CHANNEL);
        }

        uint8_t radiansToProtocolAngle(double radians, std::size_t joint_index) const
        {
            const auto lower = kLowerLimits[joint_index];
            const auto upper = kUpperLimits[joint_index];
            if (upper <= lower)
            {
                return 0;
            }

            const auto normalized = std::clamp((radians - lower) / (upper - lower), 0.0, 1.0);
            const auto angle = std::lround(normalized * static_cast<double>(FreedomArmCanProtocol::kMaxProtocolAngle));
            return static_cast<uint8_t>(std::clamp<long>(angle, 0, FreedomArmCanProtocol::kMaxProtocolAngle));
        }

        double protocolAngleToRadians(uint8_t angle, std::size_t joint_index) const
        {
            const auto lower = kLowerLimits[joint_index];
            const auto upper = kUpperLimits[joint_index];
            if (upper <= lower)
            {
                return lower;
            }

            const auto normalized = std::clamp(
                static_cast<double>(angle) / static_cast<double>(FreedomArmCanProtocol::kMaxProtocolAngle), 0.0, 1.0);
            return lower + normalized * (upper - lower);
        }

        uint8_t device_id_;
        bool is_left_hand_;
    };
} // namespace marvin_ros2_control
