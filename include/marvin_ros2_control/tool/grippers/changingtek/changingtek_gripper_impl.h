#pragma once

#include "marvin_ros2_control/tool/grippers/changingtek/changingtek_gripper.h"
#include "MarvinSDK.h"
#include <thread>
#include <chrono>
#include "rclcpp/logging.hpp"
#include "gripper_hardware_common/utils/PositionConverter.h"
#include "gripper_hardware_common/utils/TorqueConverter.h"

namespace marvin_ros2_control
{
    template<typename Config>
    ChangingtekGripper<Config>::ChangingtekGripper(Clear485Func clear_485, Send485Func send_485,
                                                   GetChDataFunc on_get_ch_data)
        : ModbusGripper(clear_485, send_485, on_get_ch_data), acceleration_set_(false), deceleration_set_(false)
    {
    }

    template<typename Config>
    bool ChangingtekGripper<Config>::initialize()
    {
        acceleration_set_ = true;
        deceleration_set_ = true;
        
        bool result = writeSingleRegister(Config::SLAVE_ID, Config::INIT_REG_ADDR, Config::POWER_ON,
                                        Config::WRITE_SINGLE_FUNCTION);
        RCLCPP_INFO(logger_, "Initializing Changingtek Gripper (slave: 0x%02X)", Config::SLAVE_ID);
        RCLCPP_INFO(logger_, "Initializing Changingtek Gripper (%d)", result);
        return result;
    }

    template<typename Config>
    bool ChangingtekGripper<Config>::move_gripper(int torque, int velocity, double normalized_pos)
    {
        using namespace gripper_hardware_common;
        
        // Convert normalized position to Modbus format (0-9000)
        uint16_t modbus_pos = PositionConverter::Changingtek90::normalizedToModbus(normalized_pos);
        int position = static_cast<int>(modbus_pos);
        
        // Convert torque: input torque (0-100, already scaled by gripper_torque_scale) 
        // is treated as normalized (0.0-1.0) and converted to Modbus torque value (0-255, 0xFF)
        // This ensures torque mapping is consistent with position mapping
        double normalized_torque = static_cast<double>(torque) / 100.0;  // Convert 0-100 to 0.0-1.0
        int modbus_torque = TorqueConverter::Changingtek90::normalizedToModbus(normalized_torque);
        
        RCLCPP_INFO(logger_, "Changingtek Gripper - pos: %.3f->%d, vel: %d, trq: %d->%d (scale: %.2f)", 
                   normalized_pos, position, velocity, torque, modbus_torque, normalized_torque);
        
        // Prepare values
        uint16_t vel_value = static_cast<uint16_t>(velocity & 0xFFFF);
        uint16_t trq_value = static_cast<uint16_t>(modbus_torque & 0xFFFF);
        uint16_t pos_low = 0x0000;
        uint16_t pos_high = static_cast<uint16_t>(position & 0xFFFF);
        
        std::vector<uint16_t> position_values = {pos_low, pos_high};
        bool result = true;
        
        // Write position (two registers) - use Config constants
        result = writeMultipleRegisters(Config::SLAVE_ID, Config::POS_REG_ADDR, position_values,
                                       Config::WRITE_FUNCTION) && result;
            
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        // Configure acceleration if not set
        if (!acceleration_set_)
        {
            result = writeSingleRegister(Config::SLAVE_ID, Config::ACCELERATION_REG, Config::DEFAULT_ACCELERATION,
                                        Config::WRITE_SINGLE_FUNCTION) && result;
            acceleration_set_ = true;
            
            // Write velocity
            result = writeSingleRegister(Config::SLAVE_ID, Config::VELOCITY_REG, vel_value,
                                        Config::WRITE_SINGLE_FUNCTION) && result;
            
            // Write torque
            result = writeSingleRegister(Config::SLAVE_ID, Config::TORQUE_REG, trq_value,
                                        Config::WRITE_SINGLE_FUNCTION) && result;
        }
        
        // Configure deceleration if not set
        if (!deceleration_set_)
        {
            result = writeSingleRegister(Config::SLAVE_ID, Config::DECELERATION_REG, Config::DEFAULT_DECELERATION,
                                        Config::WRITE_SINGLE_FUNCTION) && result;
            deceleration_set_ = true;
        }
        // Trigger movement
        result = writeSingleRegister(Config::SLAVE_ID, Config::TRIGGER_REG_ADDR, Config::TRIGGER_VALUE,
                                    Config::WRITE_SINGLE_FUNCTION) && result;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        return result;
    }

    template<typename Config>
    bool ChangingtekGripper<Config>::getStatus()
    {
        // Only send read request, don't wait for response
        // The actual status will be updated by recv_thread_func when response arrives
        return sendReadRequestAsync(Config::SLAVE_ID, Config::FEEDBACK_REG_ADDR, 2, Config::READ_FUNCTION);
    }

    template<typename Config>
    void ChangingtekGripper<Config>::updateStatusFromResponse(const std::vector<uint16_t>& registers)
    {
        if (registers.size() >= 2)
        {
            // Parse position: (high << 16) + low
            uint32_t modbus_pos = (static_cast<uint32_t>(registers[0]) << 16) | registers[1];
            // Convert Modbus position to normalized (0.0=closed, 1.0=open)
            using namespace gripper_hardware_common;
            cached_position_ = PositionConverter::Changingtek90::modbusToNormalized(modbus_pos);
            cached_velocity_ = 0;
            cached_torque_ = 0;
            status_valid_ = true;
        }
    }

    template<typename Config>
    bool ChangingtekGripper<Config>::processReadResponse(const uint8_t* data, size_t data_size,
                                                         int& torque, int& velocity, double& position)
    {
        // if (data_size < 3)
        // {
        //     return false;
        // }
        
        // Verify slave_id and function_code match expected values
        // if (data[0] != Config::SLAVE_ID || data[1] != Config::READ_FUNCTION)
        // {
        //     return false;
        // }
        
        // Parse Modbus response
        std::vector<uint16_t> registers = ModbusIO::parseModbusResponse(data, data_size, Config::SLAVE_ID, Config::READ_FUNCTION);
        if (registers.size() < 2)
        {
            std::cout << "size below 2 -----" << std::endl;
            return false;
        }
       
        // Extract position: registers[0] = 0x0414 (high), registers[1] = 0x0415 (low)
        uint32_t modbus_pos = (static_cast<uint32_t>(registers[0]) << 16) | registers[1];
        using namespace gripper_hardware_common;
        position = PositionConverter::Changingtek90::modbusToNormalized(modbus_pos);
        velocity = 0;
        torque = 0;
        
        return true;
    }

    template<typename Config>
    void ChangingtekGripper<Config>::deinitialize()
    {
        acceleration_set_ = false;
        deceleration_set_ = false;
        RCLCPP_INFO(logger_, "Changingtek Gripper deinitialized");
    }

    template<typename Config>
    void ChangingtekGripper<Config>::resetState()
    {
        acceleration_set_ = false;
        deceleration_set_ = false;
    }
} // namespace marvin_ros2_control

