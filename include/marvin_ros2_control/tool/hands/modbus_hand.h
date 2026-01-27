#pragma once

#include "marvin_ros2_control/tool/modbus_io.h"
#include "gripper_hardware_common/GripperBase.h"
#include <vector>
#include <cstdint>

namespace marvin_ros2_control
{
    /**
     * @brief Modbus Hand base class for Marvin robots
     * 
     * Combines ModbusIO (Marvin-specific) with GripperBase (shared interface).
     * This class is specific to Marvin robots due to ModbusIO dependency.
     */
    class ModbusHand : public ModbusIO, public gripper_hardware_common::GripperBase
    {
    public:
        ModbusHand(Clear485Func clear_485, Send485Func send_485,
                  GetChDataFunc on_get_ch_data = nullptr)
            : ModbusIO(clear_485, send_485, on_get_ch_data)
        {
        }
        
        virtual ~ModbusHand() = default;

        // GripperBase interface implementation
        bool initialize() override = 0;
        bool move_gripper(int torque, int velocity, double position) override = 0;
        bool getStatus() override = 0;

    protected:
        // Convenience methods that use ModbusIO (inherited from ModbusIO)
        // These are available directly without qualification
    };
} // namespace marvin_ros2_control

