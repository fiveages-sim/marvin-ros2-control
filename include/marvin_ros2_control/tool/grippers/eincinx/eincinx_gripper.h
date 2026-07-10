#pragma once

#include <cstdint>

#include "marvin_ros2_control/tool/grippers/modbus_gripper.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"

namespace marvin_ros2_control
{
    /**
     * EincinX / EPGI180 RS485 (slave 0x02).
     * Writes only when motion_stopped_; runtime reads alternate position/status.
     */
    class EincinXGripper : public ModbusGripper
    {
    public:
        EincinXGripper(Clear485Func clear_485, Send485Func send_485,
                       GetChDataFunc on_get_ch_data = nullptr);

        bool initialize() override;
        bool move_gripper(double normalized_torque, double normalized_velocity, double position) override;
        bool getStatus() override;
        bool processReadResponse(const uint8_t* data, size_t data_size,
                                 int& torque, int& velocity, double& position) override;
        bool isTargetReached() const override;
        void resetState() override;

    private:
        enum class ReadStep : uint8_t
        {
            Position,
            StatusWord,
            Version,
        };

        void queueMotionCommand(double normalized_torque, double normalized_velocity, double position_rad);
        void updatePendingWrites();
        bool advanceCommCycle();
        bool sendCurrent();
        bool sendSpeed();
        bool sendPosition();
        bool sendReadStep(ReadStep step);

        bool pending_current_write_ = false;
        bool pending_speed_write_ = false;
        bool pending_position_write_ = false;
        ReadStep active_read_step_ = ReadStep::Position;
        ReadStep last_read_step_ = ReadStep::Position;

        int32_t target_position_pulses_ = 0;
        uint32_t target_speed_pulses_ = 0;
        uint16_t target_current_ = 0;

        int32_t applied_position_pulses_ = -1;
        uint32_t applied_speed_pulses_ = 0;
        uint16_t applied_current_ = 0;

        uint16_t status_word_ = 0;
        bool motion_stopped_ = true;
        bool version_query_sent_ = false;
    };
}  // namespace marvin_ros2_control
