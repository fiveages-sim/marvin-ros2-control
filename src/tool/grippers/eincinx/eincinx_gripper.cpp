#include "marvin_ros2_control/tool/grippers/eincinx/eincinx_gripper.h"

#include <cmath>
#include <thread>
#include <chrono>
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
constexpr double kJointOpenRad = M_PI / 2.0;

// Tutorial 4.1: FC10 @6101/@4702, 32-bit big-endian (high word first).
// Close: 02 10 61 01 00 02 04 00 00 59 D8  |  Open: ... 00 00 00 00
std::vector<uint16_t> encodeSignedU32HighLow(int32_t value)
{
    const uint32_t raw = static_cast<uint32_t>(value);
    return {
        static_cast<uint16_t>((raw >> 16) & 0xFFFF),
        static_cast<uint16_t>(raw & 0xFFFF),
    };
}

int32_t decodeSignedU32(const std::vector<uint16_t>& registers)
{
    if (registers.size() < 2)
    {
        return 0;
    }
    uint32_t raw = (static_cast<uint32_t>(registers[0]) << 16) | registers[1];
    if (raw & 0x80000000u)
    {
        raw -= 0x100000000u;
    }
    return static_cast<int32_t>(raw);
}

double pulsesToJointRad(int32_t pulses)
{
    const double normalized = gripper_hardware_common::PositionConverter::EincinX::pulsesToNormalized(
        pulses, EincinXCfg::MAX_POSITION_PULSES);
    return normalized * kJointOpenRad;
}

int32_t jointRadToPulses(double joint_rad)
{
    const double normalized = std::clamp(joint_rad / kJointOpenRad, 0.0, 1.0);
    return static_cast<int32_t>(gripper_hardware_common::PositionConverter::EincinX::normalizedToPulses(
        normalized, EincinXCfg::MAX_POSITION_PULSES));
}

}  // namespace

EincinXGripper::EincinXGripper(Clear485Func clear_485, Send485Func send_485,
                                 GetChDataFunc on_get_ch_data)
    : ModbusGripper(clear_485, send_485, on_get_ch_data)
{
}

bool EincinXGripper::initialize()
{
    RCLCPP_INFO(logger_, "Initializing EincinX Gripper (slave: 0x%02X)", EincinXCfg::SLAVE_ID);
    resetState();

    bool ok = writeSingleRegister(EincinXCfg::SLAVE_ID, EincinXCfg::HOMING_CMD_REG,
                                  EincinXCfg::HOMING_CMD_VALUE, EincinXCfg::WRITE_SINGLE_FUNCTION);
    std::this_thread::sleep_for(std::chrono::milliseconds(2500));

    const uint16_t grip_current = static_cast<uint16_t>(EincinXCfg::DEFAULT_FORWARD_CURRENT);
    const uint16_t release_current = static_cast<uint16_t>(EincinXCfg::DEFAULT_REVERSE_CURRENT);
    ok = writeSingleRegister(EincinXCfg::SLAVE_ID, EincinXCfg::FORWARD_CURRENT_REG, grip_current,
                             EincinXCfg::WRITE_SINGLE_FUNCTION) && ok;
    ok = writeSingleRegister(EincinXCfg::SLAVE_ID, EincinXCfg::REVERSE_CURRENT_REG, release_current,
                             EincinXCfg::WRITE_SINGLE_FUNCTION) && ok;
    ok = writeMultipleRegisters(EincinXCfg::SLAVE_ID, EincinXCfg::SPEED_REG,
                                encodeSignedU32HighLow(static_cast<int32_t>(EincinXCfg::DEFAULT_SPEED_PULSES)),
                                EincinXCfg::WRITE_FUNCTION) && ok;
    ok = writeMultipleRegisters(EincinXCfg::SLAVE_ID, EincinXCfg::ABS_POSITION_REG,
                                encodeSignedU32HighLow(0), EincinXCfg::WRITE_FUNCTION) && ok;

    cached_grip_current_reg_ = grip_current;
    cached_release_current_reg_ = release_current;
    cached_speed_reg_ = EincinXCfg::DEFAULT_SPEED_PULSES;
    cached_position_reg_ = 0;
    return ok;
}

bool EincinXGripper::move_gripper(double normalized_torque, double normalized_velocity,
                                  double normalized_pos)
{
    using gripper_hardware_common::TorqueConverter;
    using gripper_hardware_common::VelocityConverter;

    const int32_t target_pulses = jointRadToPulses(normalized_pos);
    const int grip_current = TorqueConverter::EincinX::normalizedToGripCurrent(normalized_torque);
    const int release_current = TorqueConverter::EincinX::normalizedToReleaseCurrent(normalized_torque);
    const uint32_t speed = (normalized_velocity <= 0.0)
                               ? EincinXCfg::DEFAULT_SPEED_PULSES
                               : VelocityConverter::EincinX::normalizedToSpeed(
                                     normalized_velocity, EincinXCfg::MAX_SPEED_PULSES);

    RCLCPP_INFO(logger_,
                "EincinX FC10@6101=%d pulses, FC10@4702=%u, FC06@4307/8=%d/%d (0.01A)",
                target_pulses, speed, grip_current, release_current);

    bool result = true;
    const uint16_t grip_reg = static_cast<uint16_t>(grip_current & 0xFFFF);
    const uint16_t release_reg = static_cast<uint16_t>(release_current & 0xFFFF);

    if (grip_reg != cached_grip_current_reg_)
    {
        result = writeSingleRegister(EincinXCfg::SLAVE_ID, EincinXCfg::FORWARD_CURRENT_REG, grip_reg,
                                     EincinXCfg::WRITE_SINGLE_FUNCTION) && result;
        cached_grip_current_reg_ = grip_reg;
    }
    if (release_reg != cached_release_current_reg_)
    {
        result = writeSingleRegister(EincinXCfg::SLAVE_ID, EincinXCfg::REVERSE_CURRENT_REG, release_reg,
                                     EincinXCfg::WRITE_SINGLE_FUNCTION) && result;
        cached_release_current_reg_ = release_reg;
    }
    if (speed != cached_speed_reg_)
    {
        result = writeMultipleRegisters(EincinXCfg::SLAVE_ID, EincinXCfg::SPEED_REG,
                                        encodeSignedU32HighLow(static_cast<int32_t>(speed)),
                                        EincinXCfg::WRITE_FUNCTION) && result;
        cached_speed_reg_ = speed;
    }

    result = writeMultipleRegisters(EincinXCfg::SLAVE_ID, EincinXCfg::ABS_POSITION_REG,
                                    encodeSignedU32HighLow(target_pulses),
                                    EincinXCfg::WRITE_FUNCTION) && result;
    cached_position_reg_ = static_cast<uint32_t>(target_pulses);
    return result;
}

bool EincinXGripper::getStatus()
{
    return sendReadRequestAsync(EincinXCfg::SLAVE_ID, EincinXCfg::POSITION_FB_REG, 2,
                                EincinXCfg::READ_FUNCTION);
}

void EincinXGripper::updateStatusFromResponse(const std::vector<uint16_t>& registers)
{
    cached_position_ = pulsesToJointRad(decodeSignedU32(registers));
    cached_velocity_ = 0;
    cached_torque_ = 0;
    status_valid_ = true;
}

bool EincinXGripper::processReadResponse(const uint8_t* data, size_t data_size,
                                         int& torque, int& velocity, double& position)
{
    std::vector<uint16_t> registers = ModbusIO::parseModbusResponse(
        data, data_size, EincinXCfg::SLAVE_ID, EincinXCfg::READ_FUNCTION);
    if (registers.size() < 2)
    {
        return false;
    }

    position = pulsesToJointRad(decodeSignedU32(registers));
    velocity = 0;
    torque = 0;
    status_valid_ = true;
    return true;
}

void EincinXGripper::resetState()
{
    cached_grip_current_reg_ = 0xFFFF;
    cached_release_current_reg_ = 0xFFFF;
    cached_speed_reg_ = 0xFFFFFFFF;
    cached_position_reg_ = 0xFFFFFFFF;
}

}  // namespace marvin_ros2_control
