#pragma once

#include "marvin_ros2_control/grippers/gripper_control.h"
#include "gripper_hardware_common/ChangingtekGripper.h"

namespace marvin_ros2_control
{
    /**
     * @brief Changingtek Gripper implementation for Marvin robots
     * 
     * Uses gripper_hardware_common utilities for position conversion,
     * command change detection, and Modbus configuration.
     */
    class ChangingtekGripper : public ModbusGripper
    {
    public:
        ChangingtekGripper(Clear485Func clear_485, Send485Func send_485);
        bool initialize() override;
        bool move_gripper(int torque, int velocity, int position) override;
        bool getStatus(int& torque, int& velocity, int& position) override;
        void deinitialize() override;
        void resetState() override;

    private:
        bool acceleration_set_;
        bool deceleration_set_;
        double last_command_;  // Store last normalized command for change detection

        // Changingtek-specific registers (not in common config)
        static constexpr uint16_t INIT_REGISTER = 0x0100;
        static constexpr uint16_t INIT_VALUE = 0x0001;
        static constexpr uint16_t POSITION_REG_HIGH = 0x0103;  // POS_REG_ADDR + 1
        static constexpr uint16_t VELOCITY_REG = 0x0104;
        static constexpr uint16_t TORQUE_REG = 0x0105;
        static constexpr uint16_t ACCELERATION_REG = 0x0106;
        static constexpr uint16_t DECELERATION_REG = 0x0107;
        static constexpr uint16_t DEFAULT_ACCELERATION = 2000;
        static constexpr uint16_t DEFAULT_DECELERATION = 2000;
        static constexpr uint16_t TRIGGER_VALUE = 0x0001;
    };
} // namespace marvin_ros2_control

