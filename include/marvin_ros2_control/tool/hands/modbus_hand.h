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
        // Multi-joint hand command (normalized positions per joint).
        // Concrete hands (O7/O6/L6) should implement this.
        virtual bool move_hand(
            const std::vector<int>& torques,
            const std::vector<int>& velocities,
            const std::vector<double>& positions
        ) = 0;
        bool getStatus() override = 0;
        
        // Get hand DOF (degrees of freedom) - implemented by concrete hand types
        virtual size_t getJointCount() const = 0;
        // Map global joint name to local hand joint index (0-based, -1 if not found)
        // This allows each hand type to define its own joint ordering
        virtual int mapJointNameToIndex(const std::string& joint_name) const = 0;

        // Process Modbus read response for hand status
        // Parses Modbus response and extracts all joint positions
        // Returns true if successful, false otherwise
        // positions: output vector containing normalized positions [0.0-1.0] for all joints
        virtual bool processReadResponse(const uint8_t* data, size_t data_size,
                                        std::vector<double>& positions) = 0;

    protected:
        // Convenience methods that use ModbusIO (inherited from ModbusIO)
        // These are available directly without qualification
    };
} // namespace marvin_ros2_control

