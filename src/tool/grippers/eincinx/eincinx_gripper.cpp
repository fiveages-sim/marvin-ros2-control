#include "marvin_ros2_control/tool/grippers/eincinx/eincinx_gripper.h"

#include <chrono>
#include <thread>
#include <vector>

#include "rclcpp/logging.hpp"
#include "gripper_hardware_common/utils/PositionConverter.h"
#include "gripper_hardware_common/utils/TorqueConverter.h"
#include "gripper_hardware_common/utils/VelocityConverter.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"

namespace marvin_ros2_control
{
namespace
{
using EincinXCfg = gripper_hardware_common::ModbusConfig::EincinX;

std::vector<uint16_t> encodeU32HighLow(int32_t value)
{
    const uint32_t raw = static_cast<uint32_t>(value);
    return {
        static_cast<uint16_t>((raw >> 16) & 0xFFFF),
        static_cast<uint16_t>(raw & 0xFFFF),
    };
}

int32_t decodeSignedU32(uint16_t hi_word, uint16_t lo_word)
{
    uint32_t raw = (static_cast<uint32_t>(hi_word) << 16) | lo_word;
    if (raw & 0x80000000u)
    {
        raw -= 0x100000000u;
    }
    return static_cast<int32_t>(raw);
}

double pulsesToJointRad(int32_t pulses)
{
    return gripper_hardware_common::PositionConverter::EincinX::pulsesToPositionRad(pulses);
}

int32_t jointRadToPulses(double position_rad)
{
    return static_cast<int32_t>(
        gripper_hardware_common::PositionConverter::EincinX::positionRadToPulses(position_rad));
}

}  // namespace


EincinXGripper::EincinXGripper(Clear485Func clear_485, Send485Func send_485,
                                 GetChDataFunc on_get_ch_data)
    : ModbusGripper(clear_485, send_485, on_get_ch_data)
{
}

bool EincinXGripper::sendCurrent()
{
    return sendWriteSingleRegisterAsync(EincinXCfg::SLAVE_ID, EincinXCfg::CURRENT_REG,
                                        target_current_, EincinXCfg::WRITE_SINGLE_FUNCTION);
}

bool EincinXGripper::sendSpeed()
{
    return writeMultipleRegisters(EincinXCfg::SLAVE_ID, EincinXCfg::SPEED_REG,
                                  encodeU32HighLow(static_cast<int32_t>(target_speed_pulses_)),
                                  EincinXCfg::WRITE_FUNCTION);
}

bool EincinXGripper::sendPosition()
{
    return writeMultipleRegisters(EincinXCfg::SLAVE_ID, EincinXCfg::ABS_POSITION_REG,
                                  encodeU32HighLow(target_position_pulses_),
                                  EincinXCfg::WRITE_FUNCTION);
}

bool EincinXGripper::sendReadStep(ReadStep step)
{
    last_read_step_ = step;
    switch (step)
    {
    case ReadStep::Position:
        return sendReadRequestAsync(EincinXCfg::SLAVE_ID, EincinXCfg::POSITION_FB_REG,
                                    EincinXCfg::POSITION_FB_REGISTER_COUNT, EincinXCfg::READ_FUNCTION);
    case ReadStep::StatusWord:
        return sendReadRequestAsync(EincinXCfg::SLAVE_ID, EincinXCfg::STATUS_WORD_REG,
                                    EincinXCfg::READ_REGISTER_COUNT, EincinXCfg::READ_FUNCTION);
    default:
        return false;
    }
}

void EincinXGripper::queueMotionCommand(double normalized_torque, double normalized_velocity,
                                        double position_rad)
{
    using gripper_hardware_common::TorqueConverter;
    using gripper_hardware_common::VelocityConverter;

    target_position_pulses_ = jointRadToPulses(position_rad);
    target_current_ = static_cast<uint16_t>(
        TorqueConverter::EincinX::normalizedToGripCurrent(normalized_torque) & 0xFFFF);
    target_speed_pulses_ = (normalized_velocity <= 0.0)
                               ? EincinXCfg::DEFAULT_SPEED_PULSES
                               : VelocityConverter::EincinX::normalizedToSpeed(
                                     normalized_velocity, EincinXCfg::MAX_SPEED_PULSES);

    updatePendingWrites();
}

void EincinXGripper::updatePendingWrites()
{
    if (!motion_stopped_)
    {
        return;
    }

    if (target_current_ != applied_current_)
    {
        pending_current_write_ = true;
    }
    if (target_speed_pulses_ != applied_speed_pulses_)
    {
        pending_speed_write_ = true;
    }
    if (target_position_pulses_ != applied_position_pulses_)
    {
        pending_position_write_ = true;
    }
}

bool EincinXGripper::advanceCommCycle()
{
    if (motion_stopped_)
    {
        if (pending_current_write_)
        {
            if (!sendCurrent())
            {
                return false;
            }
            pending_current_write_ = false;
            applied_current_ = target_current_;
            RCLCPP_INFO(logger_, "EincinX current sent (FC06@4307): %u", target_current_);
            return true;
        }

        if (pending_speed_write_)
        {
            if (!sendSpeed())
            {
                return false;
            }
            pending_speed_write_ = false;
            applied_speed_pulses_ = target_speed_pulses_;
            RCLCPP_INFO(logger_, "EincinX speed sent (FC10@4702): %u", target_speed_pulses_);
            return true;
        }

        if (pending_position_write_)
        {
            if (!sendPosition())
            {
                return false;
            }
            pending_position_write_ = false;
            applied_position_pulses_ = target_position_pulses_;
            RCLCPP_INFO(logger_, "EincinX position sent (FC10@6101): %d pulses (%.6f rad)",
                        target_position_pulses_, pulsesToJointRad(target_position_pulses_));
            return true;
        }
    }

    if (!sendReadStep(active_read_step_))
    {
        return false;
    }
    active_read_step_ = (active_read_step_ == ReadStep::Position) ? ReadStep::StatusWord
                                                                  : ReadStep::Position;
    return true;
}

bool EincinXGripper::initialize()
{
    RCLCPP_INFO(logger_, "Initializing EincinX gripper (slave 0x%02X)", EincinXCfg::SLAVE_ID);
    resetState();

    auto send_fc06 = [&](uint16_t reg, uint16_t value) {
        const bool ok = sendWriteSingleRegisterAsync(EincinXCfg::SLAVE_ID, reg, value,
                                                     EincinXCfg::WRITE_SINGLE_FUNCTION);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        return ok;
    };

    bool ok = send_fc06(EincinXCfg::CONTROL_WORD_REG, EincinXCfg::ENABLE_VALUE);
    ok = send_fc06(EincinXCfg::CURRENT_REG, static_cast<uint16_t>(EincinXCfg::DEFAULT_CURRENT)) && ok;
    ok = writeMultipleRegisters(EincinXCfg::SLAVE_ID, EincinXCfg::SPEED_REG,
                                encodeU32HighLow(static_cast<int32_t>(EincinXCfg::DEFAULT_SPEED_PULSES)),
                                EincinXCfg::WRITE_FUNCTION) &&
         ok;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    ok = send_fc06(EincinXCfg::SAVE_PARAM_REG, EincinXCfg::SAVE_PARAM_VALUE) && ok;

    target_current_ = static_cast<uint16_t>(EincinXCfg::DEFAULT_CURRENT);
    target_speed_pulses_ = EincinXCfg::DEFAULT_SPEED_PULSES;
    applied_current_ = target_current_;
    applied_speed_pulses_ = target_speed_pulses_;
    motion_stopped_ = true;
    return ok;
}

bool EincinXGripper::move_gripper(double normalized_torque, double normalized_velocity,
                                  double position_rad)
{
    queueMotionCommand(normalized_torque, normalized_velocity, position_rad);
    return advanceCommCycle();
}

bool EincinXGripper::getStatus()
{
    return advanceCommCycle();
}

bool EincinXGripper::isTargetReached() const
{
    return motion_stopped_;
}

bool EincinXGripper::processReadResponse(const uint8_t* data, size_t data_size, int& torque,
                                         int& velocity, double& position)
{
    std::vector<uint16_t> registers = ModbusIO::parseModbusResponse(
        data, data_size, EincinXCfg::SLAVE_ID, EincinXCfg::READ_FUNCTION);
    if (registers.empty())
    {
        return false;
    }

    if (last_read_step_ == ReadStep::StatusWord)
    {
        const bool was_stopped = motion_stopped_;
        status_word_ = registers[0];
        motion_stopped_ = (status_word_ & EincinXCfg::STATUS_TARGET_REACHED_MASK) != 0;
        status_valid_ = true;
        position = cached_position_;
        velocity = 0;
        torque = 0;

        if (!was_stopped && motion_stopped_)
        {
            updatePendingWrites();
        }
        return true;
    }

    if (last_read_step_ != ReadStep::Position || registers.size() < 2)
    {
        return false;
    }

    const int32_t pulses = decodeSignedU32(registers[0], registers[1]);
    position = pulsesToJointRad(pulses);
    velocity = 0;
    torque = 0;
    cached_position_ = position;
    status_valid_ = true;
    return true;
}

void EincinXGripper::resetState()
{
    pending_current_write_ = false;
    pending_speed_write_ = false;
    pending_position_write_ = false;
    active_read_step_ = ReadStep::Position;
    last_read_step_ = ReadStep::Position;
    target_position_pulses_ = 0;
    target_speed_pulses_ = EincinXCfg::DEFAULT_SPEED_PULSES;
    target_current_ = static_cast<uint16_t>(EincinXCfg::DEFAULT_CURRENT);
    applied_position_pulses_ = -1;
    applied_speed_pulses_ = 0;
    applied_current_ = 0;
    status_word_ = 0;
    motion_stopped_ = true;
}

}  // namespace marvin_ros2_control
