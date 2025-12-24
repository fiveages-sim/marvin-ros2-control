#include "marvin_ros2_control/grippers/jd_gripper.h"
#include "MarvinSDK.h"
#include <thread>
#include <chrono>
#include "gripper_hardware_common/utils/PositionConverter.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
#include "gripper_hardware_common/utils/JodellCommandBuilder.h"

namespace marvin_ros2_control
{
    JDGripper::JDGripper(Clear485Func clear_485, Send485Func send_485,
                         GetChDataFunc on_get_ch_data)
        : ModbusGripper(clear_485, send_485, on_get_ch_data)
    {
    }

    bool JDGripper::initialize()
    {
        RCLCPP_INFO(logger_, "Initializing JD Gripper (slave: 0x%02X)", SLAVE_ID);
        std::vector<uint16_t> init_value_vector = {INIT_VALUE};
        return writeMultipleRegisters(SLAVE_ID, INIT_REGISTER, init_value_vector,
                                      WRITE_MULTIPLE_FUNCTION);
    }

    /// input torque is uint8_t
    /// input velocity is uint8_t
    /// input position is normalized (0.0=closed, 1.0=open)
    bool JDGripper::move_gripper(int trq_set, int vel_set, double normalized_pos)
    {
        using namespace gripper_hardware_common;
        using namespace gripper_hardware_common::ModbusConfig;
        
        // Convert normalized position to Jodell position (0-255)
        int pos_set = PositionConverter::Jodell::normalizedToJodell(normalized_pos);
        
        RCLCPP_INFO(logger_, "JD Gripper moving - normalized: %.3f (jodell: %d), vel: %d, trq: %d",
                    normalized_pos, pos_set, vel_set, trq_set);
        
        // Use JodellCommandBuilder to build the command
        auto position_values = JodellCommandBuilder::buildCommand(pos_set, trq_set, vel_set);
        
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
        
        if (data_size < 3)
        {
            return false;
        }
        
        // Verify response slave address matches request
        if (data[0] != SLAVE_ID || data[1] != READ_FUNCTION)
        {
            return false;
        }
        
        // Parse Modbus response
        std::vector<uint16_t> registers = ModbusIO::parseModbusResponse(data, data_size, SLAVE_ID, READ_FUNCTION);
        if (registers.size() < Jodell::STATUS_REG_COUNT)
        {
            return false;
        }
        
        // Extract and update torque, velocity, position
        int status_reg_high = static_cast<int>(registers[1] >> 8);
        position = PositionConverter::Jodell::jodellToNormalized(status_reg_high);
        velocity = static_cast<int>(registers[2] & 0xFF);
        torque = static_cast<int>(registers[2] >> 8);
        
        return true;
    }
} // namespace marvin_ros2_control

