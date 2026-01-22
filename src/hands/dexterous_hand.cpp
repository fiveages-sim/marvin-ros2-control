#include "marvin_ros2_control/hands/dexterous_hand.h"
#include "MarvinSDK.h"
#include <algorithm>
#include "rclcpp/logging.hpp"

namespace marvin_ros2_control
{
    DexterousHand::DexterousHand(Clear485Func clear_485, Send485Func send_485,
                                 GetChDataFunc on_get_ch_data)
        : ModbusHand(clear_485, send_485, on_get_ch_data)
    {
    }

    bool DexterousHand::initialize()
    {
        RCLCPP_INFO(logger_, "Initializing Dexterous Hand (slave: 0x%02X)", SLAVE_ID);
        // Initialize all fingers to default positions
        // Could set default torque, speed, and lock rotor parameters here
        return true;
    }

    bool DexterousHand::move_gripper(int torque, int velocity, double normalized_pos)
    {
        // For compatibility with base interface, apply same values to all 7 joints
        std::vector torques(7, torque);
        std::vector velocities(7, velocity);
        std::vector positions(7, normalized_pos);
        return move_hand(torques, velocities, positions);
    }

    bool DexterousHand::move_hand(const std::vector<int>& torques, const std::vector<int>& velocities, const std::vector<double>& positions)
    {
        // Validate input sizes - need exactly 7 values for 7 DOF
        if (torques.size() != 7 || velocities.size() != 7 || positions.size() != 7)
        {
            RCLCPP_ERROR(logger_, "Dexterous Hand: Invalid input size. Expected 7 values, got torques=%zu, velocities=%zu, positions=%zu",
                        torques.size(), velocities.size(), positions.size());
            return false;
        }

        bool result = true;

        // Joint order: 0=Thumb_Pitch, 1=Thumb_Yaw, 2=Index_Pitch, 3=Middle_Pitch, 4=Ring_Pitch, 5=Little_Pitch, 6=Thumb_Roll
        // Position registers
        std::vector position_regs = {
            THUMB_PITCH_REG,      // 0
            THUMB_YAW_REG,        // 1
            INDEX_PITCH_REG,      // 2
            MIDDLE_PITCH_REG,     // 3
            RING_PITCH_REG,       // 4
            LITTLE_PITCH_REG,     // 5
            THUMB_ROLL_REG        // 6
        };

        // Torque registers
        std::vector torque_regs = {
            THUMB_PITCH_TORQUE_REG,      // 0
            THUMB_YAW_TORQUE_REG,        // 1
            INDEX_PITCH_TORQUE_REG,      // 2
            MIDDLE_PITCH_TORQUE_REG,     // 3
            RING_PITCH_TORQUE_REG,       // 4
            LITTLE_PITCH_TORQUE_REG,     // 5
            THUMB_ROLL_TORQUE_REG        // 6
        };

        // Speed registers
        std::vector speed_regs = {
            THUMB_PITCH_SPEED_REG,      // 0
            THUMB_YAW_SPEED_REG,        // 1
            INDEX_PITCH_SPEED_REG,      // 2
            MIDDLE_PITCH_SPEED_REG,     // 3
            RING_PITCH_SPEED_REG,       // 4
            LITTLE_PITCH_SPEED_REG,     // 5
            THUMB_ROLL_SPEED_REG        // 6
        };

        // Write positions, torques, and speeds for all 7 joints
        for (size_t i = 0; i < 7; ++i)
        {
            // Convert normalized position (0.0-1.0) to hand position (0-255)
            uint8_t pos_value = static_cast<uint8_t>(std::max(0.0, std::min(1.0, positions[i])) * 255.0);
            
            // Write position
            result = writeSingleRegister(SLAVE_ID, position_regs[i], pos_value, WRITE_SINGLE_FUNCTION) && result;
            
            // Write torque (0-255 range)
            uint8_t torque_value = static_cast<uint8_t>(std::max(0, std::min(255, torques[i])));
            result = writeSingleRegister(SLAVE_ID, torque_regs[i], torque_value, WRITE_SINGLE_FUNCTION) && result;
            
            // Write speed (0-255 range)
            uint8_t speed_value = static_cast<uint8_t>(std::max(0, std::min(255, velocities[i])));
            result = writeSingleRegister(SLAVE_ID, speed_regs[i], speed_value, WRITE_SINGLE_FUNCTION) && result;
        }

        return result;
    }

    bool DexterousHand::getStatus()
    {
        // Only send read request, don't wait for response
        // The actual status will be updated by recv_thread_func when response arrives
        // Read position registers (7 registers: Thumb_Pitch, Thumb_Yaw, Index_Pitch, Middle_Pitch, Ring_Pitch, Little_Pitch, Thumb_Roll)
        return sendReadRequestAsync(SLAVE_ID, THUMB_PITCH_REG, 7, READ_FUNCTION);
    }

    void DexterousHand::deinitialize()
    {
        RCLCPP_INFO(logger_, "Dexterous Hand deinitialized");
    }

    void DexterousHand::resetState()
    {
        // Reset hand state if needed
    }

    bool DexterousHand::setFingerPosition(uint8_t finger_id, uint8_t position)
    {
        uint16_t reg = getPositionRegister(finger_id);
        if (reg == 0xFFFF)
        {
            return false;
        }
        return writeSingleRegister(SLAVE_ID, reg, position, WRITE_SINGLE_FUNCTION);
    }

    bool DexterousHand::setFingerTorque(uint8_t finger_id, uint8_t torque)
    {
        uint16_t reg = getTorqueRegister(finger_id);
        if (reg == 0xFFFF)
        {
            return false;
        }
        return writeSingleRegister(SLAVE_ID, reg, torque, WRITE_SINGLE_FUNCTION);
    }

    bool DexterousHand::setFingerSpeed(uint8_t finger_id, uint8_t speed)
    {
        uint16_t reg = getSpeedRegister(finger_id);
        if (reg == 0xFFFF)
        {
            return false;
        }
        return writeSingleRegister(SLAVE_ID, reg, speed, WRITE_SINGLE_FUNCTION);
    }

    bool DexterousHand::getFingerPosition(uint8_t finger_id, uint8_t& position)
    {
        uint16_t reg = getPositionRegister(finger_id);
        if (reg == 0xFFFF)
        {
            return false;
        }
        std::vector<uint16_t> response = readRegisters(SLAVE_ID, reg, 1, READ_FUNCTION);
        if (response.size() >= 1)
        {
            position = static_cast<uint8_t>(response[0] & 0xFF);
            return true;
        }
        return false;
    }

    bool DexterousHand::getFingerTorque(uint8_t finger_id, uint8_t& torque)
    {
        uint16_t reg = getTorqueRegister(finger_id);
        if (reg == 0xFFFF)
        {
            return false;
        }
        std::vector<uint16_t> response = readRegisters(SLAVE_ID, reg, 1, READ_FUNCTION);
        if (response.size() >= 1)
        {
            torque = static_cast<uint8_t>(response[0] & 0xFF);
            return true;
        }
        return false;
    }

    bool DexterousHand::getFingerSpeed(uint8_t finger_id, uint8_t& speed)
    {
        uint16_t reg = getSpeedRegister(finger_id);
        if (reg == 0xFFFF)
        {
            return false;
        }
        std::vector<uint16_t> response = readRegisters(SLAVE_ID, reg, 1, READ_FUNCTION);
        if (response.size() >= 1)
        {
            speed = static_cast<uint8_t>(response[0] & 0xFF);
            return true;
        }
        return false;
    }

    uint16_t DexterousHand::getPositionRegister(uint8_t finger_id) const
    {
        // Finger IDs: 0=Thumb_Pitch, 1=Thumb_Yaw, 2=Index_Pitch, 3=Middle_Pitch, 4=Ring_Pitch, 5=Little_Pitch, 6=Thumb_Roll
        switch (finger_id)
        {
            case 0: return THUMB_PITCH_REG;
            case 1: return THUMB_YAW_REG;
            case 2: return INDEX_PITCH_REG;
            case 3: return MIDDLE_PITCH_REG;
            case 4: return RING_PITCH_REG;
            case 5: return LITTLE_PITCH_REG;
            case 6: return THUMB_ROLL_REG;
            default: return 0xFFFF;  // Invalid
        }
    }

    uint16_t DexterousHand::getTorqueRegister(uint8_t finger_id) const
    {
        switch (finger_id)
        {
            case 0: return THUMB_PITCH_TORQUE_REG;
            case 1: return THUMB_YAW_TORQUE_REG;
            case 2: return INDEX_PITCH_TORQUE_REG;
            case 3: return MIDDLE_PITCH_TORQUE_REG;
            case 4: return RING_PITCH_TORQUE_REG;
            case 5: return LITTLE_PITCH_TORQUE_REG;
            case 6: return THUMB_ROLL_TORQUE_REG;
            default: return 0xFFFF;  // Invalid
        }
    }

    uint16_t DexterousHand::getSpeedRegister(uint8_t finger_id) const
    {
        switch (finger_id)
        {
            case 0: return THUMB_PITCH_SPEED_REG;
            case 1: return THUMB_YAW_SPEED_REG;
            case 2: return INDEX_PITCH_SPEED_REG;
            case 3: return MIDDLE_PITCH_SPEED_REG;
            case 4: return RING_PITCH_SPEED_REG;
            case 5: return LITTLE_PITCH_SPEED_REG;
            case 6: return THUMB_ROLL_SPEED_REG;
            default: return 0xFFFF;  // Invalid
        }
    }
} // namespace marvin_ros2_control

