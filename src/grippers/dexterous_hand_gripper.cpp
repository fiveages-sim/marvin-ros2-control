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

    bool DexterousHandGripper::processReadResponse(const uint8_t* data, size_t data_size,
                                                   int& torque, int& velocity, double& position)
    {
        // 响应协议: 从站号 + 功能代码 + 字节数 + 7个寄存器数据 (每个2字节) + CRC
        // 例如: 01 03 0E 00 80 00 80 00 80 00 80 00 80 00 80 00 80 [CRC]
        // 01: 从站号
        // 03: 功能代码 03（读取保持寄存器）
        // 0E: 数据字节数（7个寄存器 × 2个字节 = 14个字节 = 0x0E）
        // 00 80: 寄存器0的值（位置值，0-255）
        // ... 其他6个寄存器
        
        if (data_size < 5)
        {
            return false;
        }

        // 打印原始读取数据
        std::string hex_str;
        for (size_t i = 0; i < data_size; ++i)
        {
            char hex[4];
            snprintf(hex, sizeof(hex), "%02X ", data[i]);
            hex_str += hex;
        }
        RCLCPP_INFO(logger_, "📥 Dexterous Hand Raw Response (%zu bytes): %s", data_size, hex_str.c_str());

        // 验证响应
        if (data[0] != SLAVE_ID || data[1] != READ_FUNCTION)
        {
            RCLCPP_WARN(logger_, "Response mismatch: slave=0x%02X (expected 0x%02X), func=0x%02X (expected 0x%02X)",
                       data[0], SLAVE_ID, data[1], READ_FUNCTION);
            return false;
        }

        // 解析寄存器数据
        uint8_t byte_count = data[2];
        if (byte_count != 14)  // 7个寄存器 × 2字节 = 14字节
        {
            RCLCPP_WARN(logger_, "Unexpected byte count: %d (expected 14)", byte_count);
            return false;
        }

        if (data_size < static_cast<size_t>(3 + byte_count + 2))  // 3字节头部 + 数据 + 2字节CRC
        {
            RCLCPP_WARN(logger_, "Insufficient data size: %zu (expected at least %zu)", 
                       data_size, static_cast<size_t>(3 + byte_count + 2));
            return false;
        }

        // 解析7个寄存器值
        std::vector<uint16_t> registers;
        registers.reserve(7);
        for (size_t i = 0; i < 7; ++i)
        {
            size_t idx = 3 + i * 2;
            uint16_t value = static_cast<uint16_t>(data[idx] << 8) | data[idx + 1];
            registers.push_back(value);
        }

        // 更新状态
        updateStatusFromResponse(registers);

        // 返回第一个关节的值（用于兼容性）
        position = joint_positions_[0];
        velocity = static_cast<int>(joint_velocities_[0] * 255.0);
        torque = static_cast<int>(joint_efforts_[0] * 255.0);

        return true;
    }

    void DexterousHandGripper::updateStatusFromResponse(const std::vector<uint16_t>& registers)
    {
        if (registers.size() < 7)
        {
            RCLCPP_WARN(logger_, "Insufficient registers: got %zu, expected 7", registers.size());
            return;
        }

        // 打印解析后的寄存器值
        RCLCPP_INFO(logger_, "📥 Dexterous Hand Parsed Registers:");
        for (size_t i = 0; i < 7; ++i)
        {
            uint8_t raw_pos = static_cast<uint8_t>(registers[i] & 0xFF);
            // 将原始值 (0-255) 转换为归一化值 (0.0-1.0)
            joint_positions_[i] = static_cast<double>(raw_pos) / 255.0;
            joint_velocities_[i] = 0.0;  // 速度暂时设为0，可以从其他寄存器读取
            joint_efforts_[i] = 0.0;     // 力矩暂时设为0，可以从其他寄存器读取
            
            RCLCPP_INFO(logger_, "  Joint[%zu]: raw=0x%02X (0x%04X), normalized=%.3f",
                       i, raw_pos, registers[i], joint_positions_[i]);
        }

        RCLCPP_INFO(logger_, "  Parsed positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                   joint_positions_[0], joint_positions_[1], joint_positions_[2],
                   joint_positions_[3], joint_positions_[4], joint_positions_[5],
                   joint_positions_[6]);

        status_valid_ = true;
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

