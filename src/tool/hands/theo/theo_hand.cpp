#include "marvin_ros2_control/tool/hands/theo/theo_hand.h"

#include <algorithm>
#include <cmath>

#include "rclcpp/logging.hpp"

namespace marvin_ros2_control
{
    const std::array<const char*, TheoHandSTD16A::kJointCount> TheoHandSTD16A::kJointSuffixes = {
        "thumb_joint1", "thumb_joint2", "thumb_joint3", "thumb_joint4",
        "index_joint1", "index_joint2", "index_joint3",
        "middle_joint1", "middle_joint2", "middle_joint3",
        "ring_joint1", "ring_joint2", "ring_joint3",
        "pinky_joint1", "pinky_joint2", "pinky_joint3"
    };

    const std::array<double, TheoHandSTD16A::kJointCount> TheoHandSTD16A::kLowerLimits = {
        -0.001, 0.0, 0.0, -35.0 * TheoHandSTD16A::kPi / 180.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0
    };

    const std::array<double, TheoHandSTD16A::kJointCount> TheoHandSTD16A::kUpperLimits = {
        0.04, TheoHandSTD16A::kPi / 2.0, TheoHandSTD16A::kPi / 2.0, 85.0 * TheoHandSTD16A::kPi / 180.0,
        TheoHandSTD16A::kPi / 2.0, TheoHandSTD16A::kPi / 2.0, TheoHandSTD16A::kPi / 2.0,
        TheoHandSTD16A::kPi / 2.0, TheoHandSTD16A::kPi / 2.0, TheoHandSTD16A::kPi / 2.0,
        TheoHandSTD16A::kPi / 2.0, TheoHandSTD16A::kPi / 2.0, TheoHandSTD16A::kPi / 2.0,
        TheoHandSTD16A::kPi / 2.0, TheoHandSTD16A::kPi / 2.0, TheoHandSTD16A::kPi / 2.0
    };

    TheoHandSTD16A::TheoHandSTD16A(Clear485Func clear_485, Send485Func send_485,
                                   GetChDataFunc on_get_ch_data, long channel,
                                   uint8_t slave_id)
        : ModbusHand(clear_485, send_485, on_get_ch_data, channel),
          slave_id_(slave_id)
    {
    }

    bool TheoHandSTD16A::initialize()
    {
        RCLCPP_INFO(logger_,
                    "Initializing TheoHand STD16A (slave: 0x%02X, channel: %ld): FC06 @0x%04X = 0x%04X",
                    slave_id_, channel_, kControlWordRegister, kEnableControlWord);
        return writeSingleRegister(slave_id_, kControlWordRegister, kEnableControlWord, kWriteSingleFunction);
    }

    bool TheoHandSTD16A::move_gripper(double normalized_torque,
                                      double normalized_velocity,
                                      double position)
    {
        (void)normalized_torque;
        (void)normalized_velocity;

        const double close_ratio = std::clamp(position, 0.0, 1.0);
        std::vector<double> positions(kJointCount, 0.0);
        positions[1] = close_ratio * kUpperLimits[1];
        positions[2] = close_ratio * kUpperLimits[2];
        for (size_t i = 4; i < kJointCount; ++i)
        {
            positions[i] = close_ratio * kUpperLimits[i];
        }
        return move_hand({}, {}, positions);
    }

    bool TheoHandSTD16A::move_hand(const std::vector<double>& torques,
                                   const std::vector<double>& velocities,
                                   const std::vector<double>& positions)
    {
        (void)torques;
        (void)velocities;

        if (positions.size() < kJointCount)
        {
            RCLCPP_ERROR(logger_, "TheoHandSTD16A: expected %zu joint positions, got %zu",
                         kJointCount, positions.size());
            return false;
        }

        std::vector<uint16_t> registers(kJointCount, 0);
        for (size_t i = 0; i < kJointCount; ++i)
        {
            registers[i] = radiansToProtocol(positions[i], kLowerLimits[i], kUpperLimits[i]);
        }

        return writeMultipleRegisters(slave_id_, kTargetPositionRegister, registers, kWriteMultipleFunction);
    }

    bool TheoHandSTD16A::getStatus()
    {
        return sendReadRequestAsync(slave_id_, kStatusWordRegister,
                                    static_cast<uint16_t>(kJointCount + 1), kReadFunction);
    }

    int TheoHandSTD16A::mapJointNameToIndex(const std::string& joint_name) const
    {
        const std::string n = lower(joint_name);
        for (size_t i = 0; i < kJointCount; ++i)
        {
            const std::string suffix = kJointSuffixes[i];
            if (n.size() >= suffix.size() &&
                n.compare(n.size() - suffix.size(), suffix.size(), suffix) == 0)
            {
                return static_cast<int>(i);
            }
        }
        return -1;
    }

    bool TheoHandSTD16A::processReadResponse(const uint8_t* data, size_t data_size,
                                             std::vector<double>& positions)
    {
        if (data_size < 5 || data[0] != slave_id_)
        {
            return false;
        }

        const std::vector<uint16_t> registers =
            parseModbusResponse(data, data_size, slave_id_, kReadFunction);
        if (registers.size() < kJointCount)
        {
            return false;
        }

        size_t position_offset = 0;
        if (registers.size() >= kJointCount + 1)
        {
            const uint16_t status_word = registers[0];
            position_offset = 1;
            if (!status_word_initialized_ || status_word != last_status_word_)
            {
                status_word_initialized_ = true;
                last_status_word_ = status_word;
                RCLCPP_DEBUG(logger_,
                             "TheoHand STD16A StatusWord=0x%04X ready=%u switched_on=%u enabled=%u fault=%u warning=%u target_reached=%u internal_limit=%u",
                             status_word,
                             (status_word >> 0) & 0x1,
                             (status_word >> 1) & 0x1,
                             (status_word >> 2) & 0x1,
                             (status_word >> 3) & 0x1,
                             (status_word >> 7) & 0x1,
                             (status_word >> 10) & 0x1,
                             (status_word >> 11) & 0x1);
            }
        }

        positions.assign(kJointCount, 0.0);
        for (size_t i = 0; i < kJointCount; ++i)
        {
            const int raw = static_cast<int>(static_cast<int16_t>(registers[position_offset + i]));
            positions[i] = protocolToRadians(raw, kLowerLimits[i], kUpperLimits[i]);
        }
        return true;
    }

    uint16_t TheoHandSTD16A::radiansToProtocol(double position, double lower, double upper)
    {
        if (upper <= lower)
        {
            return 0;
        }
        const double ratio = std::clamp((position - lower) / (upper - lower), 0.0, 1.0);
        return static_cast<uint16_t>(std::lround(ratio * static_cast<double>(kProtocolPositionMax)));
    }

    double TheoHandSTD16A::protocolToRadians(int raw_position, double lower, double upper)
    {
        if (upper <= lower)
        {
            return lower;
        }
        const double raw = std::clamp(static_cast<double>(raw_position), 0.0,
                                      static_cast<double>(kProtocolPositionMax));
        return lower + (raw / static_cast<double>(kProtocolPositionMax)) * (upper - lower);
    }

    std::string TheoHandSTD16A::lower(const std::string& value)
    {
        std::string out = value;
        std::transform(out.begin(), out.end(), out.begin(), ::tolower);
        return out;
    }
} // namespace marvin_ros2_control
