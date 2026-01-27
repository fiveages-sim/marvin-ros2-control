#pragma once

#include "marvin_ros2_control/tool/modbus_io.h"
#include "gripper_hardware_common/GripperBase.h"
#include <vector>
#include <cstdint>

namespace marvin_ros2_control
{
    /**
     * @brief Modbus Gripper base class for Marvin robots
     * 
     * Combines ModbusIO (Marvin-specific) with GripperBase (shared interface).
     * This class is specific to Marvin robots due to ModbusIO dependency.
     */
    class ModbusGripper : public ModbusIO, public gripper_hardware_common::GripperBase
    {
    public:
        ModbusGripper(Clear485Func clear_485, Send485Func send_485,
                     GetChDataFunc on_get_ch_data = nullptr)
            : ModbusIO(clear_485, send_485, on_get_ch_data)
        {
        }
        
        virtual ~ModbusGripper() = default;

        // GripperBase interface implementation
        bool initialize() override = 0;
        bool move_gripper(int torque, int velocity, double position) override = 0;
        bool getStatus() override = 0;

        // Additional methods for Modbus grippers
        virtual void updateStatusFromResponse(const std::vector<uint16_t>& registers) 
        {
            (void)registers;  // Suppress unused parameter warning
        }
        virtual bool processReadResponse(const uint8_t* data, size_t data_size,
                                        int& torque, int& velocity, double& position)
        {
            (void)data;
            (void)data_size;
            (void)torque;
            (void)velocity;
            (void)position;
            return false;
        }

    protected:
        // Cached status values (updated by updateStatusFromResponse)
        double cached_position_ = 0.0;
        int cached_velocity_ = 0;
        int cached_torque_ = 0;
        bool status_valid_ = false;
        
        // Static logger (inherited from ModbusIO)
        using ModbusIO::logger_;
    };
} // namespace marvin_ros2_control

