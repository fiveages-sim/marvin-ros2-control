#include "marvin_ros2_control/grippers/jd_gripper.h"
#include "MarvinSDK.h"
#include <thread>
#include <chrono>

namespace marvin_ros2_control
{
    JDGripper::JDGripper(Clear485Func clear_485, Send485Func send_485)
        : ModbusGripper(clear_485, send_485)
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
    /// input position is uint8_t
    bool JDGripper::move_gripper(int trq_set, int vel_set, int pos_set)
    {
        RCLCPP_INFO(logger_, "JD Gripper moving - pos: %d, vel: %d, trq: %d",
                    pos_set, vel_set, trq_set);
        /// fill in data section
        uint16_t trigger = 0x09;
        uint16_t position = pos_set << 8;
        uint16_t speed_force = trq_set << 8 | vel_set;
        std::vector<uint16_t> position_values = {};
        position_values.push_back(trigger);
        position_values.push_back(position);
        position_values.push_back(speed_force);

        bool result = true;
        // Write position (two registers)
        result = writeMultipleRegisters(SLAVE_ID, POSITION_REG, position_values,
                                        WRITE_MULTIPLE_FUNCTION);

        return result;
    }

    bool JDGripper::getStatus(int& /*torque*/, int& /*velocity*/, int& /*position*/)
    {
        std::vector<uint16_t> response = readRegisters(SLAVE_ID, STATUS_REG, READ_REG_NUM,
                                                       READ_FUNCTION);

        bool success = true;
        // bool success = response.size() >= 2;

        // if (success) {
        //     position = (static_cast<int32_t>(response[0]) << 16) | response[1];
        //     velocity = 0;
        //     torque = 0;
        // }
        (void)response;  // Suppress unused variable warning
        return success;
    }
} // namespace marvin_ros2_control

