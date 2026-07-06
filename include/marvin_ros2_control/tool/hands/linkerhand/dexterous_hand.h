#pragma once

#include "marvin_ros2_control/tool/hands/modbus_hand.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
#include "gripper_hardware_common/utils/PositionConverter.h"
#include "rclcpp/logging.hpp"
#include <vector>
#include <cstdint>
#include <cmath>
#include <limits>
#include <algorithm>
#include <string>

using namespace gripper_hardware_common;

namespace marvin_ros2_control
{
    /**
     * @brief Product type enum for different LinkerHand models
     */
    enum class LinkerHandProduct
    {
        O7,  // 7-DOF hand
        O6,  // 6-DOF hand
        L6   // 6-DOF hand (different model)
    };

    /**
     * @brief Traits class for LinkerHand products
     * 
     * Specialize this for different hand products to define:
     * - Joint count (DOF)
     * - Product name for logging
     */
    template<LinkerHandProduct Product>
    struct LinkerHandProductTraits;

    template<>
    struct LinkerHandProductTraits<LinkerHandProduct::O7>
    {
        static constexpr size_t JOINT_COUNT = 7;
        static constexpr const char* PRODUCT_NAME = "O7";
        using Converter = PositionConverter::LinkerHandO7;
    };

    template<>
    struct LinkerHandProductTraits<LinkerHandProduct::O6>
    {
        static constexpr size_t JOINT_COUNT = 6;
        static constexpr const char* PRODUCT_NAME = "O6";
        using Converter = PositionConverter::LinkerHandO6;
    };

    template<>
    struct LinkerHandProductTraits<LinkerHandProduct::L6>
    {
        static constexpr size_t JOINT_COUNT = 6;
        static constexpr const char* PRODUCT_NAME = "L6";
        using Converter = PositionConverter::LinkerHandL6;
    };

    /**
     * @brief Template-based Dexterous Hand implementation for Marvin robots
     * 
     * Common implementation for all LinkerHand products (O7, O6, L6).
     * Configuration is specified as a template parameter.
     * 
     * @tparam Product Product type (O7, O6, L6)
     */
    template<LinkerHandProduct Product>
    class DexterousHandTemplate : public ModbusHand
    {
    public:
        using Traits = LinkerHandProductTraits<Product>;
        using Converter = typename Traits::Converter;
        static constexpr size_t JOINT_COUNT = Traits::JOINT_COUNT;

        DexterousHandTemplate(Clear485Func clear_485, Send485Func send_485,
                             GetChDataFunc on_get_ch_data = nullptr,
                             bool is_left_hand = true)
            : ModbusHand(clear_485, send_485, on_get_ch_data),
              slave_id_(is_left_hand ? ModbusConfig::LinkerHand::LEFT_HAND_SLAVE_ID 
                                     : ModbusConfig::LinkerHand::RIGHT_HAND_SLAVE_ID)
        {
            last_applied_torques_.assign(JOINT_COUNT, std::numeric_limits<double>::quiet_NaN());
            last_applied_velocities_.assign(JOINT_COUNT, std::numeric_limits<double>::quiet_NaN());
        }

    protected:
        uint8_t slave_id_;  // Modbus slave ID (0x28 for left, 0x27 for right)

        // O6/L6: pos 0-5, torque 6-11, speed 12-17 (read/write 0-17 when dynamics change)
        // O7:    pos 0-6, torque 7-13, speed 14-20
        static constexpr uint16_t torqueRegStart()
        {
            return (JOINT_COUNT == 7) ? static_cast<uint16_t>(7) : static_cast<uint16_t>(6);
        }
        static constexpr uint16_t speedRegStart()
        {
            return (JOINT_COUNT == 7) ? static_cast<uint16_t>(14) : static_cast<uint16_t>(12);
        }
        static constexpr uint16_t fullInputRegisterCount()
        {
            return static_cast<uint16_t>(speedRegStart() + JOINT_COUNT);
        }

        std::vector<double> last_applied_torques_;
        std::vector<double> last_applied_velocities_;
        bool pending_full_register_io_{false};

        bool dynamicsChanged(const std::vector<double>& torques,
                             const std::vector<double>& velocities) const
        {
            constexpr double kEps = 0.001;
            for (size_t i = 0; i < JOINT_COUNT; ++i)
            {
                if (std::isnan(last_applied_torques_[i]) ||
                    std::abs(torques[i] - last_applied_torques_[i]) > kEps)
                {
                    return true;
                }
                if (std::isnan(last_applied_velocities_[i]) ||
                    std::abs(velocities[i] - last_applied_velocities_[i]) > kEps)
                {
                    return true;
                }
            }
            return false;
        }

        bool initialize() override
        {
            RCLCPP_INFO(logger_, "Initializing LinkerHand %s (%zu-DOF, slave: 0x%02X)", 
                       Traits::PRODUCT_NAME, JOINT_COUNT, slave_id_);
            return true;
        }

        bool move_gripper(double normalized_torque, double normalized_velocity, double normalized_pos) override
        {
            std::vector<double> torques(JOINT_COUNT, std::clamp(normalized_torque, 0.0, 1.0));
            std::vector<double> velocities(JOINT_COUNT, std::clamp(normalized_velocity, 0.0, 1.0));
            std::vector<double> positions(JOINT_COUNT, normalized_pos);
            return move_hand(torques, velocities, positions);
        }

        bool move_hand(
            const std::vector<double>& torques,
            const std::vector<double>& velocities,
            const std::vector<double>& positions
        ) override
        {
            // Validate input sizes
            if (torques.size() != JOINT_COUNT || velocities.size() != JOINT_COUNT || positions.size() != JOINT_COUNT)
            {
                RCLCPP_ERROR(logger_, "LinkerHand %s: Invalid input size. Expected %zu values, got torques=%zu, velocities=%zu, positions=%zu",
                            Traits::PRODUCT_NAME, JOINT_COUNT, torques.size(), velocities.size(), positions.size());
                return false;
            }

            // Convert positions to raw values (0-255) and arrange in Modbus register order
            // According to modbus_ros2_control implementation: use FC 16 (Write Multiple Holding Registers)
            // Only write position registers (0x0000-0x0005 for 6-DOF, 0x0000-0x0006 for 7-DOF)
            // Note: positions[] array order matches mapJointNameToIndex() return order, which should match Modbus register order
            std::vector<uint16_t> raw_positions(JOINT_COUNT, 0);

            for (size_t i = 0; i < JOINT_COUNT; ++i)
            {
                const std::string joint_name = Converter::getJointNameByIndex(i);
                const double clamped_rad = Converter::clampToLimits(positions[i], joint_name);
                raw_positions[i] = Converter::rad_to_modbus_raw(positions[i], joint_name);

                RCLCPP_DEBUG(logger_,
                          "  Register[%zu] (%s): rad=%.4f -> raw=%u, torque=%.2f, velocity=%.2f",
                          i, joint_name.c_str(), clamped_rad, raw_positions[i], torques[i], velocities[i]);
            }

            const bool write_dynamics = dynamicsChanged(torques, velocities);

            if (write_dynamics)
            {
                // O6: 一次 FC16 写 0-17（位置+力矩+速度）；O7: 0-20
                const uint16_t n_regs = fullInputRegisterCount();
                std::vector<uint16_t> block(n_regs, 0);
                for (size_t i = 0; i < JOINT_COUNT; ++i)
                {
                    block[i] = raw_positions[i];
                    block[torqueRegStart() + i] = static_cast<uint16_t>(
                        std::lround(std::clamp(torques[i], 0.0, 1.0) * 255.0));
                    block[speedRegStart() + i] = static_cast<uint16_t>(
                        std::lround(std::clamp(velocities[i], 0.0, 1.0) * 255.0));
                }
                if (!writeMultipleRegisters(slave_id_, ModbusConfig::LinkerHand::THUMB_PITCH_REG, block,
                                            ModbusConfig::LinkerHand::WRITE_FUNCTION))
                {
                    return false;
                }
                last_applied_torques_ = torques;
                last_applied_velocities_ = velocities;
                pending_full_register_io_ = true;
                {
                    static rclcpp::Clock kLogClock(RCL_STEADY_TIME);
                    RCLCPP_INFO_THROTTLE(logger_, kLogClock, 2000,
                                        "LinkerHand %s (0x%02X): FC16 write regs 0-%u (pos+torque+velocity)",
                                        Traits::PRODUCT_NAME, slave_id_, static_cast<unsigned>(n_regs - 1));
                }
            }
            else
            {
                if (!writeMultipleRegisters(slave_id_,
                                            ModbusConfig::LinkerHand::THUMB_PITCH_REG,
                                            raw_positions,
                                            ModbusConfig::LinkerHand::WRITE_FUNCTION))
                {
                    return false;
                }
                pending_full_register_io_ = false;
                if (ModbusIO::isDebugEnabled())
                {
                    std::string raw_str;
                    for (size_t i = 0; i < JOINT_COUNT; ++i)
                    {
                        if (i > 0) raw_str += ", ";
                        raw_str += std::to_string(raw_positions[i]);
                    }
                    RCLCPP_DEBUG(logger_, "LinkerHand %s (0x%02X): FC16 position write raw=[%s]",
                                 Traits::PRODUCT_NAME, slave_id_, raw_str.c_str());
                }
                else
                {
                    static rclcpp::Clock kLogClock(RCL_STEADY_TIME);
                    RCLCPP_INFO_THROTTLE(logger_, kLogClock, 2000,
                                        "LinkerHand %s (0x%02X): FC16 position write (%zu joints)",
                                        Traits::PRODUCT_NAME, slave_id_, JOINT_COUNT);
                }
            }
            return true;
        }

        bool getStatus() override
        {
            const uint16_t count = pending_full_register_io_
                ? fullInputRegisterCount()
                : static_cast<uint16_t>(JOINT_COUNT);
            return sendReadRequestAsync(slave_id_,
                                       ModbusConfig::LinkerHand::THUMB_PITCH_REG,
                                       count,
                                       ModbusConfig::LinkerHand::READ_FUNCTION);
        }

        bool processReadResponse(const uint8_t* data, size_t data_size,
                                std::vector<double>& positions) override
        {
            positions.clear();
            
            if (data_size < 5) // minimal RTU frame size
            {
                RCLCPP_WARN(logger_, "LinkerHand %s: Response too short: %zu bytes (min 5)",
                           Traits::PRODUCT_NAME, data_size);
                return false;
            }
            
            // Debug: log received data (first 20 bytes)
            std::string hex_str;
            for (size_t i = 0; i < std::min(data_size, static_cast<size_t>(20)); ++i)
            {
                char buf[4];
                snprintf(buf, sizeof(buf), "%02X ", data[i]);
                hex_str += buf;
            }
            if (ModbusIO::isDebugEnabled())
            {
                RCLCPP_INFO(logger_, "LinkerHand %s (slave 0x%02X): Received %zu bytes: %s...",
                            Traits::PRODUCT_NAME, slave_id_, data_size, hex_str.c_str());
            }
            
            // Parse Modbus response using inherited parseModbusResponse
            std::vector<uint16_t> registers = parseModbusResponse(
                data, data_size, 
                slave_id_, 
                ModbusConfig::LinkerHand::READ_FUNCTION
            );
            
            const size_t expected_regs = pending_full_register_io_
                ? static_cast<size_t>(fullInputRegisterCount()) : JOINT_COUNT;

            if (registers.size() < expected_regs)
            {
                const uint8_t actual_fc = (data_size >= 2) ? data[1] : 0;
                if (actual_fc == ModbusConfig::LinkerHand::WRITE_FUNCTION)
                {
                    RCLCPP_DEBUG(logger_,
                                 "LinkerHand %s: ignored write ACK (FC 0x10) while expecting read",
                                 Traits::PRODUCT_NAME);
                }
                else
                {
                    static rclcpp::Clock kReadWarnClock(RCL_STEADY_TIME);
                    RCLCPP_WARN_THROTTLE(logger_, kReadWarnClock, 2000,
                                        "LinkerHand %s: read expected %zu registers, got %zu (fc=0x%02X)",
                                        Traits::PRODUCT_NAME, expected_regs, registers.size(), actual_fc);
                }
                return false;
            }
            
            // Convert register values to radians via PositionConverter
            positions.reserve(JOINT_COUNT);
            for (size_t i = 0; i < JOINT_COUNT; ++i)
            {
                const uint8_t raw_pos = static_cast<uint8_t>(registers[i] & 0xFF);
                const std::string joint_name = Converter::getJointNameByIndex(i);
                positions.push_back(Converter::modbus_raw_to_rad(raw_pos, joint_name));
            }

            if (pending_full_register_io_ && registers.size() >= expected_regs)
            {
                pending_full_register_io_ = false;
            }
            
            return true;
        }

        size_t getJointCount() const override
        {
            return JOINT_COUNT;
        }

        int mapJointNameToIndex(const std::string& joint_name) const override
        {
            std::string name_lower = joint_name;
            std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);
            
            // Joint order mapping based on hand type (O7 vs O6/L6)
            if constexpr (JOINT_COUNT == 7)  // O7
            {
                if (name_lower.find("thumb_joint1") != std::string::npos) return 0;
                if (name_lower.find("thumb_joint2") != std::string::npos) return 1;
                if (name_lower.find("thumb_joint3") != std::string::npos) return 6;
                if (name_lower.find("index_joint") != std::string::npos)  return 2;
                if (name_lower.find("middle_joint") != std::string::npos) return 3;
                if (name_lower.find("ring_joint") != std::string::npos)   return 4;
                if (name_lower.find("pinky_joint") != std::string::npos)  return 5;
            }
            else  // O6/L6 (6-DOF)
            {
                if (name_lower.find("thumb_joint1") != std::string::npos) return 0;
                if (name_lower.find("thumb_joint2") != std::string::npos) return 1;
                if (name_lower.find("index_joint") != std::string::npos)  return 2;
                if (name_lower.find("middle_joint") != std::string::npos) return 3;
                if (name_lower.find("ring_joint") != std::string::npos)   return 4;
                if (name_lower.find("pinky_joint") != std::string::npos)  return 5;
            }
            return -1;
        }

        void deinitialize() override
        {
            RCLCPP_INFO(logger_, "LinkerHand %s deinitialized", Traits::PRODUCT_NAME);
        }

        void resetState() override
        {
            last_applied_torques_.assign(JOINT_COUNT, std::numeric_limits<double>::quiet_NaN());
            last_applied_velocities_.assign(JOINT_COUNT, std::numeric_limits<double>::quiet_NaN());
            pending_full_register_io_ = false;
        }

        // Additional methods for individual finger control
        bool setFingerPosition(uint8_t finger_id, uint8_t position)
        {
            if (finger_id >= JOINT_COUNT)
            {
                return false;
            }
            return writeSingleRegister(slave_id_, 
                                      ModbusConfig::LinkerHand::POSITION_REGS[finger_id], position, 
                                      ModbusConfig::LinkerHand::WRITE_SINGLE_FUNCTION);
        }

        bool setFingerTorque(uint8_t finger_id, uint8_t torque)
        {
            if (finger_id >= JOINT_COUNT)
            {
                return false;
            }
            return writeSingleRegister(slave_id_, 
                                      ModbusConfig::LinkerHand::TORQUE_REGS[finger_id], torque, 
                                      ModbusConfig::LinkerHand::WRITE_SINGLE_FUNCTION);
        }

        bool setFingerSpeed(uint8_t finger_id, uint8_t speed)
        {
            if (finger_id >= JOINT_COUNT)
            {
                return false;
            }
            return writeSingleRegister(slave_id_, 
                                      ModbusConfig::LinkerHand::SPEED_REGS[finger_id], speed, 
                                      ModbusConfig::LinkerHand::WRITE_SINGLE_FUNCTION);
        }

        bool getFingerPosition(uint8_t finger_id, uint8_t& position)
        {
            if (finger_id >= JOINT_COUNT)
            {
                return false;
            }
            std::vector<uint16_t> response = readRegisters(slave_id_, 
                                                           ModbusConfig::LinkerHand::POSITION_REGS[finger_id], 
                                                           1, 
                                                           ModbusConfig::LinkerHand::READ_FUNCTION);
            if (response.size() >= 1)
            {
                position = static_cast<uint8_t>(response[0] & 0xFF);
                return true;
            }
            return false;
        }

        bool getFingerTorque(uint8_t finger_id, uint8_t& torque)
        {
            if (finger_id >= JOINT_COUNT)
            {
                return false;
            }
            std::vector<uint16_t> response = readRegisters(slave_id_, 
                                                           ModbusConfig::LinkerHand::TORQUE_REGS[finger_id], 
                                                           1, 
                                                           ModbusConfig::LinkerHand::READ_FUNCTION);
            if (response.size() >= 1)
            {
                torque = static_cast<uint8_t>(response[0] & 0xFF);
                return true;
            }
            return false;
        }

        bool getFingerSpeed(uint8_t finger_id, uint8_t& speed)
        {
            if (finger_id >= JOINT_COUNT)
            {
                return false;
            }
            std::vector<uint16_t> response = readRegisters(slave_id_, 
                                                           ModbusConfig::LinkerHand::SPEED_REGS[finger_id], 
                                                           1, 
                                                           ModbusConfig::LinkerHand::READ_FUNCTION);
            if (response.size() >= 1)
            {
                speed = static_cast<uint8_t>(response[0] & 0xFF);
                return true;
            }
            return false;
        }
    };

    // Type aliases for specific variants
    using DexterousHandO7 = DexterousHandTemplate<LinkerHandProduct::O7>;
    using DexterousHandO6 = DexterousHandTemplate<LinkerHandProduct::O6>;
    using DexterousHandL6 = DexterousHandTemplate<LinkerHandProduct::L6>;

    // Backward compatibility alias
    using DexterousHand = DexterousHandO7;
} // namespace marvin_ros2_control
