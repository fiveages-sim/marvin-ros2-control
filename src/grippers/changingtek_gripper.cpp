#include "marvin_ros2_control/grippers/changingtek_gripper.h"
#include "MarvinSDK.h"
#include <thread>
#include <chrono>

using namespace gripper_hardware_common;
using namespace ModbusConfig::Changingtek90;

namespace marvin_ros2_control
{
    ChangingtekGripper::ChangingtekGripper(Clear485Func clear_485, Send485Func send_485)
        : ModbusGripper(clear_485, send_485), acceleration_set_(false), deceleration_set_(false), last_command_(-1.0)
    {
    }

    bool ChangingtekGripper::initialize()
    {
        acceleration_set_ = false;
        deceleration_set_ = false;
        last_command_ = -1.0;

        RCLCPP_INFO(logger_, "Initializing Changingtek Gripper (slave: 0x%02X)", DEFAULT_SLAVE_ID);

        return writeSingleRegister(DEFAULT_SLAVE_ID, INIT_REGISTER, INIT_VALUE,
                                   WRITE_SINGLE_FUNCTION);
    }

    bool ChangingtekGripper::move_gripper(int torque, int velocity, int position)
    {
        // Position is expected to be in Modbus format (0-9000)
        // Convert to normalized for change detection
        double normalized_pos = PositionConverter::Changingtek90::modbusToNormalized(static_cast<uint32_t>(position));
        
        // Use CommandChangeDetector to check if command has changed
        if (!CommandChangeDetector::hasChanged(normalized_pos, last_command_))
        {
            return true; // Command unchanged, skip sending
        }

        RCLCPP_INFO(logger_, "Changingtek Gripper moving - pos: %d (normalized: %.3f), vel: %d, trq: %d",
                    position, normalized_pos, velocity, torque);

        // Prepare values
        uint16_t vel_value = static_cast<uint16_t>(velocity & 0xFFFF);
        uint16_t trq_value = static_cast<uint16_t>(torque & 0xFFFF);
        
        // Position format: high register = 0x0000, low register = position value
        uint16_t pos_low = 0x0000;
        uint16_t pos_high = static_cast<uint16_t>(position & 0xFFFF);

        std::vector position_values = {pos_low, pos_high};

        bool result = true;

        // Step 1: Write control mode = 0 (register 0x0102)
        result = writeSingleRegister(DEFAULT_SLAVE_ID, POS_REG_ADDR, 0x0000,
                                     WRITE_SINGLE_FUNCTION) && result;

        // Step 2: Write position value (registers 0x0102-0x0103)
        result = writeMultipleRegisters(DEFAULT_SLAVE_ID, POS_REG_ADDR, position_values,
                                        WRITE_FUNCTION) && result;

        // Configure acceleration if not set
        if (!acceleration_set_)
        {
            result = writeSingleRegister(DEFAULT_SLAVE_ID, ACCELERATION_REG, DEFAULT_ACCELERATION,
                                         WRITE_SINGLE_FUNCTION) && result;
            acceleration_set_ = true;

            // Write velocity
            result = writeSingleRegister(DEFAULT_SLAVE_ID, VELOCITY_REG, vel_value,
                                         WRITE_SINGLE_FUNCTION) && result;

            // Write torque
            result = writeSingleRegister(DEFAULT_SLAVE_ID, TORQUE_REG, trq_value,
                                         WRITE_SINGLE_FUNCTION) && result;
        }

        // Configure deceleration if not set
        if (!deceleration_set_)
        {
            result = writeSingleRegister(DEFAULT_SLAVE_ID, DECELERATION_REG, DEFAULT_DECELERATION,
                                         WRITE_SINGLE_FUNCTION) && result;
            deceleration_set_ = true;
        }

        // Step 3: Trigger movement (register 0x0108 = 1)
        result = writeSingleRegister(DEFAULT_SLAVE_ID, TRIGGER_REG_ADDR, TRIGGER_VALUE,
                                     WRITE_SINGLE_FUNCTION) && result;

        last_command_ = normalized_pos;
        return result;
    }

    bool ChangingtekGripper::getStatus(int& torque, int& velocity, int& position)
    {
        std::vector<uint16_t> response = readRegisters(DEFAULT_SLAVE_ID, FEEDBACK_REG_ADDR, 2,
                                                       READ_FUNCTION);

        bool success = response.size() >= 2;

        if (success)
        {
            // Parse position: (high << 16) + low
            uint32_t modbus_pos = static_cast<uint32_t>(response[0]) << 16 | response[1];
            position = static_cast<int>(modbus_pos);
            velocity = 0;
            torque = 0;
        }

        return success;
    }

    void ChangingtekGripper::deinitialize()
    {
        acceleration_set_ = false;
        deceleration_set_ = false;
        last_command_ = -1.0;
        RCLCPP_INFO(logger_, "Changingtek Gripper deinitialized");
    }

    void ChangingtekGripper::resetState()
    {
        acceleration_set_ = false;
        deceleration_set_ = false;
        last_command_ = -1.0;
    }
} // namespace marvin_ros2_control

