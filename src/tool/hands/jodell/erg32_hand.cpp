#include "marvin_ros2_control/tool/hands/jodell/erg32_hand.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "rclcpp/logging.hpp"

using namespace gripper_hardware_common;
using namespace gripper_hardware_common::ModbusConfig;

namespace marvin_ros2_control
{
    namespace
    {
        std::string lower(const std::string& s)
        {
            std::string out = s;
            std::transform(out.begin(), out.end(), out.begin(), ::tolower);
            return out;
        }
    } // namespace

    ERG32Hand::ERG32Hand(Clear485Func clear_485, Send485Func send_485, GetChDataFunc on_get_ch_data)
        : ModbusHand(clear_485, send_485, on_get_ch_data)
    {
    }

    uint8_t ERG32Hand::toByte(double normalized)
    {
        return static_cast<uint8_t>(std::lround(std::clamp(normalized, 0.0, 1.0) * 255.0));
    }

    bool ERG32Hand::initialize()
    {
        // ERG32 manual V1.6: one FC16 frame activates grip + rotate (0x03E8, 0x03E9).
        // 09 10 03 E8 00 02 04 00 01 00 01 <CRC>
        RCLCPP_INFO(logger_,
                    "Initializing ERG32 hand (slave: 0x%02X): FC16 @0x%04X x%u (grip+rotate rACT=1)",
                    ERG32::SLAVE_ID, ERG32::CTRL_REG_BASE, ERG32::CTRL_REG_COUNT);
        const std::vector<uint16_t> values(ERG32::CTRL_REG_COUNT, ERG32::ACTIVATE_VALUE);
        return writeMultipleRegisters(ERG32::SLAVE_ID, ERG32::CTRL_REG_BASE, values,
                                      ERG32::WRITE_FUNCTION);
    }

    bool ERG32Hand::move_gripper(double normalized_torque, double normalized_velocity, double position)
    {
        std::vector<double> torques(2, normalized_torque);
        std::vector<double> velocities(2, normalized_velocity);
        std::vector<double> positions = {0.0, position};
        return move_hand(torques, velocities, positions);
    }

    bool ERG32Hand::move_hand(const std::vector<double>& torques,
                              const std::vector<double>& velocities,
                              const std::vector<double>& positions)
    {
        if (positions.size() < 2)
        {
            RCLCPP_ERROR(logger_, "ERG32Hand: expected 2 joint positions, got %zu", positions.size());
            return false;
        }

        const double rotate_rad = std::clamp(positions[0], -kPi, kPi);
        const double grip_m = std::clamp(positions[1], 0.0, kGripperOpenM);
        const int16_t rot_deg = static_cast<int16_t>(std::lround(rotate_rad * 180.0 / kPi));

        const double grip_norm = grip_m / kGripperOpenM;
        const uint8_t grip_pos = static_cast<uint8_t>(255 - std::lround(std::clamp(grip_norm, 0.0, 1.0) * 255.0));
        const uint8_t grip_vel = toByte(velocities.size() > 1 ? velocities[1] : 1.0);
        const uint8_t grip_trq = toByte(torques.size() > 1 ? torques[1] : 1.0);
        const uint8_t rot_vel = toByte(velocities.empty() ? 1.0 : velocities[0]);
        const uint8_t rot_trq = toByte(torques.empty() ? 1.0 : torques[0]);

        std::vector<uint16_t> block(6, 0);
        block[0] = static_cast<uint16_t>(grip_pos | (static_cast<uint16_t>(grip_vel) << 8));
        block[1] = static_cast<uint16_t>(ERG32::GRIP_TRIGGER | (static_cast<uint16_t>(grip_trq) << 8));
        block[2] = static_cast<uint16_t>(rot_deg);
        block[3] = static_cast<uint16_t>(rot_vel | (static_cast<uint16_t>(rot_trq) << 8));
        block[4] = 0;
        block[5] = static_cast<uint16_t>(ERG32::ROTATE_ABS_TRIGGER);

        RCLCPP_DEBUG(logger_,
                     "ERG32 move: rotate_deg=%d vel=%u trq=%u | grip_pos=%u vel=%u trq=%u",
                     static_cast<int>(rot_deg), rot_vel, rot_trq, grip_pos, grip_vel, grip_trq);

        return writeMultipleRegisters(ERG32::SLAVE_ID, ERG32::GRIP_PARAM_REG, block, ERG32::WRITE_FUNCTION);
    }

    bool ERG32Hand::getStatus()
    {
        return sendReadRequestAsync(ERG32::SLAVE_ID, ERG32::STATUS_REG_ADDR,
                                    ERG32::STATUS_REG_COUNT, ERG32::READ_FUNCTION);
    }

    int ERG32Hand::mapJointNameToIndex(const std::string& joint_name) const
    {
        const std::string n = lower(joint_name);
        if (n.find("rotate") != std::string::npos)
        {
            return 0;
        }
        if (n.find("gripper") != std::string::npos)
        {
            return 1;
        }
        return -1;
    }

    bool ERG32Hand::processReadResponse(const uint8_t* data, size_t data_size,
                                        std::vector<double>& positions)
    {
        if (data_size < 3 || data[0] != ERG32::SLAVE_ID || data[1] != ERG32::READ_FUNCTION)
        {
            return false;
        }

        const std::vector<uint16_t> registers =
            ModbusIO::parseModbusResponse(data, data_size, ERG32::SLAVE_ID, ERG32::READ_FUNCTION);
        if (registers.size() < 5)
        {
            return false;
        }

        const uint8_t grip_raw = static_cast<uint8_t>(registers[2] & 0xFF);
        const double grip_norm = 1.0 - (static_cast<double>(grip_raw) / 255.0);
        const double grip_m = std::clamp(grip_norm, 0.0, 1.0) * kGripperOpenM;
        const int16_t rotate_deg = static_cast<int16_t>(registers[4]);
        const double rotate_rad = rotate_deg * kPi / 180.0;
        positions = {rotate_rad, grip_m};
        return true;
    }
} // namespace marvin_ros2_control
