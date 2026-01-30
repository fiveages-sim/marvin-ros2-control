#pragma once

#include "marvin_ros2_control/tool/grippers/modbus_gripper.h"
#include "gripper_hardware_common/ChangingtekGripper.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"

namespace marvin_ros2_control
{
    /**
     * @brief Template Changingtek Gripper implementation for Marvin robots
     * 
     * Common implementation for all Changingtek Gripper variants.
     * Configuration is specified as a template parameter from ModbusConfig.
     * 
     * @tparam Config Modbus configuration struct (e.g., ModbusConfig::Changingtek90C, ModbusConfig::Changingtek90D)
     */
    template<typename Config>
    class ChangingtekGripper : public ModbusGripper
    {
    public:
        ChangingtekGripper(Clear485Func clear_485, Send485Func send_485,
                           GetChDataFunc on_get_ch_data = nullptr);
        bool initialize() override;
        bool move_gripper(int torque, int velocity, double position) override;
        bool getStatus() override;
        void updateStatusFromResponse(const std::vector<uint16_t>& registers) override;
        bool processReadResponse(const uint8_t* data, size_t data_size,
                                int& torque, int& velocity, double& position) override;
        void deinitialize() override;
        void resetState() override;

    private:
        bool acceleration_set_;
        bool deceleration_set_;
    };

    // Type aliases for specific variants
    using ChangingtekGripper90C = ChangingtekGripper<gripper_hardware_common::ModbusConfig::Changingtek90C>;
    using ChangingtekGripper90D = ChangingtekGripper<gripper_hardware_common::ModbusConfig::Changingtek90D>;
} // namespace marvin_ros2_control

// Include template implementation
#include "marvin_ros2_control/tool/grippers/changingtek/changingtek_gripper_impl.h"
