#include "marvin_ros2_control/tool/grippers/jodell/jd_gripper.h"
#include "MarvinSDK.h"
#include <thread>
#include <chrono>
#include <algorithm>
#include "rclcpp/logging.hpp"
#include "gripper_hardware_common/utils/PositionConverter.h"
#include "gripper_hardware_common/utils/TorqueConverter.h"
#include "gripper_hardware_common/utils/VelocityConverter.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
#include "gripper_hardware_common/utils/JodellCommandBuilder.h"

using namespace gripper_hardware_common;
using namespace ModbusConfig;

namespace marvin_ros2_control
{
    JDGripper::JDGripper(Clear485Func clear_485, Send485Func send_485,
                         GetChDataFunc on_get_ch_data)
        : ModbusGripper(clear_485, send_485, on_get_ch_data)
    {
    }

    bool JDGripper::initialize()
    {
        RCLCPP_INFO(logger_, "Initializing JD Gripper (slave: 0x%02X)", Jodell::SLAVE_ADDRESS);
        std::vector<uint16_t> init_value_vector = {Jodell::INIT_VALUE};
        return writeMultipleRegisters(Jodell::SLAVE_ADDRESS, Jodell::INIT_REGISTER, init_value_vector,
                                      Jodell::WRITE_FUNCTION);
    }

    /// Normalized torque and velocity [0,1]; position normalized (0.0=closed, 1.0=open).
    /// Velocity maps linearly to Modbus low byte 0–255.
    bool JDGripper::move_gripper(double normalized_torque, double normalized_velocity, double normalized_pos)
    {
        using namespace gripper_hardware_common;
        using namespace gripper_hardware_common::ModbusConfig;
        
        // Convert normalized position to Jodell position (0-255)
        int pos_set = PositionConverter::Jodell::normalizedToJodell(normalized_pos);
        
        int jodell_torque = TorqueConverter::Jodell::normalizedToJodell(normalized_torque);
        const int vel_set = VelocityConverter::Jodell::normalizedToLowByte(normalized_velocity);
        
        RCLCPP_INFO(logger_, "JD Gripper - pos: %.3f->%d, vel_norm: %.3f->%d, trq_norm: %.3f->%d",
                    normalized_pos, pos_set, normalized_velocity, vel_set, normalized_torque, jodell_torque);
        
        // Use JodellCommandBuilder to build the command with converted torque
        auto position_values = JodellCommandBuilder::buildCommand(pos_set, jodell_torque, vel_set);
        
        // Write position command using ModbusConfig constants
        bool result = writeMultipleRegisters(Jodell::SLAVE_ADDRESS, Jodell::POSITION_REG_ADDR, 
                                            position_values, Jodell::WRITE_FUNCTION);

        return result;
    }

    bool JDGripper::getStatus()
    {
        using namespace gripper_hardware_common;
        using namespace gripper_hardware_common::ModbusConfig;
        
        // Only send read request, don't wait for response
        // The actual status will be updated by recv_thread_func when response arrives
        return sendReadRequestAsync(Jodell::SLAVE_ADDRESS, Jodell::STATUS_REG_ADDR, 
                                    Jodell::STATUS_REG_COUNT, Jodell::READ_FUNCTION);
    }

    void JDGripper::updateStatusFromResponse(const std::vector<uint16_t>& registers)
    {
        using namespace gripper_hardware_common;
        using namespace gripper_hardware_common::ModbusConfig;
        
        if (registers.size() >= Jodell::STATUS_REG_COUNT)
        {
            // Extract position from status register (high byte of register 1)
            int status_reg_high = static_cast<int>(registers[1] >> 8);
            // Convert Jodell position to normalized (0.0=closed, 1.0=open)
            cached_position_ = PositionConverter::Jodell::jodellToNormalized(status_reg_high);
            // Extract velocity from status register (low byte of register 2)
            cached_velocity_ = static_cast<int>(registers[2] & 0xFF);
            // Extract torque from status register (high byte of register 2)
            cached_torque_ = static_cast<int>(registers[2] >> 8);
            status_valid_ = true;
        }
    }

    bool JDGripper::processReadResponse(const uint8_t* data, size_t data_size,
                                       int& torque, int& velocity, double& position)
    {
        using namespace gripper_hardware_common;
        using namespace gripper_hardware_common::ModbusConfig;

        char hex_str[512] = {0};
        size_t hex_pos = 0;
        constexpr size_t hex_str_limit = sizeof(hex_str) - 3;
        for (size_t i = 0; i < data_size && i < 256 && hex_pos < hex_str_limit; i++)
        {
            hex_pos += snprintf(hex_str + hex_pos, sizeof(hex_str) - hex_pos, "%02X ", data[i]);
        }
        if (hex_pos > 0 && hex_str[hex_pos - 1] == ' ')
            hex_str[hex_pos - 1] = '\0';
        RCLCPP_DEBUG(logger_, "JD Gripper RX: %s (size=%zu)", hex_str, data_size);
        
        if (data_size < 3)
        {
            return false;
        }
        
        // Verify response slave address matches request
        if (data[0] != Jodell::SLAVE_ADDRESS || data[1] != Jodell::READ_FUNCTION)
        {
            return false;
        }
        
        // Parse Modbus response
        std::vector<uint16_t> registers = ModbusIO::parseModbusResponse(data, data_size, Jodell::SLAVE_ADDRESS, Jodell::READ_FUNCTION);
        if (registers.size() < Jodell::STATUS_REG_COUNT)
        {
            return false;
        }
        
        // Extract status byte from 5th byte (index 4) of Modbus response
        // Format: [Slave ID][Func Code][Byte Count][Reg07D0_High][Reg07D0_Low=Status]...
        //         [0x09]   [0x04]     [0x06]       [0x00]        [0xB1/0xF1] <- status byte
        if (data_size >= 5)
        {
            cached_status_byte_ = data[4];  // 5th byte is the status byte
        }
        else
        {
            cached_status_byte_ = static_cast<uint8_t>(registers[0] & 0xFF);  // Fallback to register parsing
        }
        
        // Extract and update torque, velocity, position
        int status_reg_high = static_cast<int>(registers[1] >> 8);
        position = PositionConverter::Jodell::jodellToNormalized(status_reg_high);
        velocity = static_cast<int>(registers[2] & 0xFF);
        torque = static_cast<int>(registers[2] >> 8);
        
        status_valid_ = true;
        
        return true;
    }

    bool JDGripper::isTargetReached() const
    {
        if (!status_valid_)
            return false;
        
        const uint8_t gOBJ = (cached_status_byte_ >> 6) & 0x03;
        return (gOBJ == 0x01 || gOBJ == 0x02 || gOBJ == 0x03);
    }
} // namespace marvin_ros2_control

