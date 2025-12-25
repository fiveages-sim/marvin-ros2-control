#include "marvin_ros2_control/grippers/dexterous_hand_gripper.h"
#include "MarvinSDK.h"
#include <algorithm>

namespace marvin_ros2_control
{
    DexterousHandGripper::DexterousHandGripper(Clear485Func clear_485, Send485Func send_485,
                                               GetChDataFunc on_get_ch_data)
        : ModbusHand(clear_485, send_485, on_get_ch_data)
    {
    }

    bool DexterousHandGripper::initialize()
    {
        RCLCPP_INFO(logger_, "Initializing Dexterous Hand Gripper (slave: 0x%02X)", SLAVE_ID);
        // Initialize all fingers to default positions
        // Could set default torque, speed, and lock rotor parameters here
        return true;
    }

    bool DexterousHandGripper::move_gripper(int torque, int velocity, double normalized_pos)
    {
        // For compatibility with base interface, apply same values to all 7 joints
        std::vector torques(7, torque);
        std::vector velocities(7, velocity);
        std::vector positions(7, normalized_pos);
        return move_gripper(torques, velocities, positions);
    }

    bool DexterousHandGripper::move_gripper(const std::vector<int>& torques, const std::vector<int>& velocities, const std::vector<double>& positions)
    {
        // Validate input sizes - need exactly 7 values for 7 DOF
        if (torques.size() != 7 || velocities.size() != 7 || positions.size() != 7)
        {
            RCLCPP_ERROR(logger_, "Dexterous Hand Gripper: Invalid input size. Expected 7 values, got torques=%zu, velocities=%zu, positions=%zu",
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
            // Convert normalized position (0.0-1.0) to finger position (0-255)
            uint8_t pos = std::clamp(static_cast<uint8_t>(positions[i] * 255.0), static_cast<uint8_t>(0), static_cast<uint8_t>(255));
            uint8_t trq = std::clamp(static_cast<uint8_t>(torques[i]), static_cast<uint8_t>(0), static_cast<uint8_t>(255));
            uint8_t vel = std::clamp(static_cast<uint8_t>(velocities[i]), static_cast<uint8_t>(0), static_cast<uint8_t>(255));

            // Write position
            result = writeSingleRegister(SLAVE_ID, position_regs[i], pos, WRITE_SINGLE_FUNCTION) && result;
            // Write torque
            result = writeSingleRegister(SLAVE_ID, torque_regs[i], trq, WRITE_SINGLE_FUNCTION) && result;
            // Write speed
            result = writeSingleRegister(SLAVE_ID, speed_regs[i], vel, WRITE_SINGLE_FUNCTION) && result;
        }

        RCLCPP_INFO(logger_, "Dexterous Hand Gripper: Set 7-DOF positions=[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]",
                   positions[0], positions[1], positions[2], positions[3], positions[4], positions[5], positions[6]);

        return result;
    }

    bool DexterousHandGripper::getStatus()
    {
        // Only send read request, don't wait for response
        // The actual status will be updated by recv_thread_func when response arrives
        // Read position registers (7 registers: Thumb_Pitch, Thumb_Yaw, Index_Pitch, Middle_Pitch, Ring_Pitch, Little_Pitch, Thumb_Roll)
        return sendReadRequestAsync(SLAVE_ID, THUMB_PITCH_REG, 7, READ_FUNCTION);
    }

    void DexterousHandGripper::deinitialize()
    {
        RCLCPP_INFO(logger_, "Dexterous Hand Gripper deinitialized");
    }

    void DexterousHandGripper::resetState()
    {
        // Reset state if needed
    }

    bool DexterousHandGripper::setFingerPosition(uint8_t finger_id, uint8_t position)
    {
        uint16_t reg = getPositionRegister(finger_id);
        if (reg == 0xFFFF)  // Invalid finger ID
        {
            return false;
        }
        position = std::clamp(position, static_cast<uint8_t>(0), static_cast<uint8_t>(255));
        return writeSingleRegister(SLAVE_ID, reg, position, WRITE_SINGLE_FUNCTION);
    }

    bool DexterousHandGripper::setFingerTorque(uint8_t finger_id, uint8_t torque)
    {
        uint16_t reg = getTorqueRegister(finger_id);
        if (reg == 0xFFFF)  // Invalid finger ID
        {
            return false;
        }
        torque = std::clamp(torque, static_cast<uint8_t>(0), static_cast<uint8_t>(255));
        return writeSingleRegister(SLAVE_ID, reg, torque, WRITE_SINGLE_FUNCTION);
    }

    bool DexterousHandGripper::setFingerSpeed(uint8_t finger_id, uint8_t speed)
    {
        uint16_t reg = getSpeedRegister(finger_id);
        if (reg == 0xFFFF)  // Invalid finger ID
        {
            return false;
        }
        speed = std::clamp(speed, static_cast<uint8_t>(0), static_cast<uint8_t>(255));
        return writeSingleRegister(SLAVE_ID, reg, speed, WRITE_SINGLE_FUNCTION);
    }

    bool DexterousHandGripper::getFingerPosition(uint8_t finger_id, uint8_t& position)
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

    bool DexterousHandGripper::getFingerTorque(uint8_t finger_id, uint8_t& torque)
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

    bool DexterousHandGripper::getFingerSpeed(uint8_t finger_id, uint8_t& speed)
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

    uint16_t DexterousHandGripper::getPositionRegister(uint8_t finger_id) const
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

    uint16_t DexterousHandGripper::getTorqueRegister(uint8_t finger_id) const
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

    uint16_t DexterousHandGripper::getSpeedRegister(uint8_t finger_id) const
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

