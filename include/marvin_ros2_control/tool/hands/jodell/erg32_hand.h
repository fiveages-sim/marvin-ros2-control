#pragma once

#include "marvin_ros2_control/tool/hands/modbus_hand.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
#include "gripper_hardware_common/utils/PositionConverter.h"

namespace marvin_ros2_control
{
    /** Jodell ERG32-150: 2-DOF (rotate + grip) on Marvin 485 bus. */
    class ERG32Hand : public ModbusHand
    {
    public:
        ERG32Hand(Clear485Func clear_485, Send485Func send_485,
                  GetChDataFunc on_get_ch_data = nullptr);

        bool initialize() override;
        bool move_gripper(double normalized_torque, double normalized_velocity, double position) override;
        bool move_hand(const std::vector<double>& torques,
                       const std::vector<double>& velocities,
                       const std::vector<double>& positions) override;
        bool getStatus() override;
        size_t getJointCount() const override { return 2; }
        int mapJointNameToIndex(const std::string& joint_name) const override;
        bool processReadResponse(const uint8_t* data, size_t data_size,
                                 std::vector<double>& positions) override;

    private:
        static uint8_t toByte(double normalized);
        static constexpr double kGripperOpenM = 0.016;
        static constexpr double kPi = 3.1415926535897931;
    };
} // namespace marvin_ros2_control
