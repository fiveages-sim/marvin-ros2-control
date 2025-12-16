#pragma once

#include "marvin_ros2_control/grippers/gripper_control.h"

namespace marvin_ros2_control
{
    /**
     * @brief JD Gripper (RG75) implementation for Marvin robots
     */
    class JDGripper : public ModbusGripper
    {
    public:
        JDGripper(Clear485Func clear_485, Send485Func send_485);
        bool initialize() override;
        bool move_gripper(int torque, int velocity, int position) override;
        bool getStatus(int& torque, int& velocity, int& position) override;

    private:
        // GH-specific Modbus configuration
        static constexpr uint8_t SLAVE_ID = 0x09;
        static constexpr uint8_t WRITE_SINGLE_FUNCTION = 0x06; // Write Single Register
        static constexpr uint8_t WRITE_MULTIPLE_FUNCTION = 0x10; // Write Multiple Registers
        /// initialize gripper
        static constexpr uint16_t INIT_REGISTER = 0x03E8;
        static constexpr uint16_t INIT_VALUE = 0x0000;
        static constexpr uint16_t INIT_REGISTER_NUM = 0x0002;

        /// write position
        static constexpr uint16_t POSITION_REG = 0x03E8;

        /// read position
        static constexpr uint8_t READ_FUNCTION = 0x04; // Read Holding Registers
        static constexpr uint16_t STATUS_REG = 0x07D0;
        static constexpr uint16_t READ_REG_NUM = 0x0003;
    };
} // namespace marvin_ros2_control

