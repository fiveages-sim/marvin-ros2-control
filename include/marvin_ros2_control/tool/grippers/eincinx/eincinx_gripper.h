#pragma once

#include "marvin_ros2_control/tool/grippers/modbus_gripper.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"

namespace marvin_ros2_control
{
    /** EincinX electric gripper (RS485 Modbus, grip axis slave ID 2). */
    class EincinXGripper : public ModbusGripper
    {
    public:
        EincinXGripper(Clear485Func clear_485, Send485Func send_485,
                       GetChDataFunc on_get_ch_data = nullptr);

        bool initialize() override;
        bool move_gripper(double normalized_torque, double normalized_velocity, double position) override;
        bool getStatus() override;
        void updateStatusFromResponse(const std::vector<uint16_t>& registers) override;
        bool processReadResponse(const uint8_t* data, size_t data_size,
                                 int& torque, int& velocity, double& position) override;
        void resetState() override;

    private:
        uint16_t cached_grip_current_reg_ = 0xFFFF;
        uint16_t cached_release_current_reg_ = 0xFFFF;
        uint32_t cached_speed_reg_ = 0xFFFFFFFF;
        uint32_t cached_position_reg_ = 0xFFFFFFFF;
    };
}  // namespace marvin_ros2_control
