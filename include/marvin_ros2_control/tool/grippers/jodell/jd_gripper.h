#pragma once

#include "marvin_ros2_control/tool/grippers/modbus_gripper.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"

namespace marvin_ros2_control
{
    /**
     * @brief JD Gripper (RG75) implementation for Marvin robots
     */
    class JDGripper : public ModbusGripper
    {
    public:
        JDGripper(Clear485Func clear_485, Send485Func send_485,
                  GetChDataFunc on_get_ch_data = nullptr);
        bool initialize() override;
        bool move_gripper(int torque, int velocity, double position) override;
        bool getStatus() override;
        void updateStatusFromResponse(const std::vector<uint16_t>& registers) override;
        bool processReadResponse(const uint8_t* data, size_t data_size, 
                                int& torque, int& velocity, double& position) override;

    private:
        // Modbus configuration will be used directly from ModbusConfig::Jodell in implementation
    };
} // namespace marvin_ros2_control

