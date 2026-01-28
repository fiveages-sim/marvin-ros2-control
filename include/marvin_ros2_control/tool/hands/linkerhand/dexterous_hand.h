#pragma once

#include "marvin_ros2_control/tool/hands/modbus_hand.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
#include "rclcpp/logging.hpp"
#include <vector>
#include <cstdint>
#include <algorithm>

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
    };

    template<>
    struct LinkerHandProductTraits<LinkerHandProduct::O6>
    {
        static constexpr size_t JOINT_COUNT = 6;
        static constexpr const char* PRODUCT_NAME = "O6";
    };

    template<>
    struct LinkerHandProductTraits<LinkerHandProduct::L6>
    {
        static constexpr size_t JOINT_COUNT = 6;
        static constexpr const char* PRODUCT_NAME = "L6";
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
        static constexpr size_t JOINT_COUNT = Traits::JOINT_COUNT;

        DexterousHandTemplate(Clear485Func clear_485, Send485Func send_485,
                             GetChDataFunc on_get_ch_data = nullptr,
                             bool is_left_hand = true)
            : ModbusHand(clear_485, send_485, on_get_ch_data),
              slave_id_(is_left_hand ? ModbusConfig::LinkerHand::LEFT_HAND_SLAVE_ID 
                                     : ModbusConfig::LinkerHand::RIGHT_HAND_SLAVE_ID)
        {
        }

    protected:
        uint8_t slave_id_;  // Modbus slave ID (0x28 for left, 0x27 for right)

        bool initialize() override
        {
            RCLCPP_INFO(logger_, "Initializing LinkerHand %s (%zu-DOF, slave: 0x%02X)", 
                       Traits::PRODUCT_NAME, JOINT_COUNT, slave_id_);
            return true;
        }

        bool move_gripper(int torque, int velocity, double normalized_pos) override
        {
            // For compatibility with base interface, apply same values to all joints
            std::vector<int> torques(JOINT_COUNT, torque);
            std::vector<int> velocities(JOINT_COUNT, velocity);
            std::vector<double> positions(JOINT_COUNT, normalized_pos);
            return move_hand(torques, velocities, positions);
        }

        bool move_hand(
            const std::vector<int>& torques,
            const std::vector<int>& velocities,
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
            
            // Log detailed joint information before conversion
            RCLCPP_INFO(logger_, "LinkerHand %s (Slave ID: 0x%02X): Converting %zu joint positions", 
                       Traits::PRODUCT_NAME, slave_id_, JOINT_COUNT);
            
            for (size_t i = 0; i < JOINT_COUNT; ++i)
            {
                // Convert normalized position (0.0-1.0) to hand position (0-255)
                // Note: positions[i] should be normalized (0.0-1.0), not radians
                double normalized = std::max(0.0, std::min(1.0, positions[i]));
                // Convert to 0-255 range (inverted: 255 = closed/flexed, 0 = open/extended)
                // Protocol: 0 = bend/close, 255 = straight/open (from modbus_ros2_control)
                raw_positions[i] = static_cast<uint16_t>(255 - std::round(normalized * 255.0));
                
                // Log each joint's conversion details
                RCLCPP_INFO(logger_, "  Register[%zu]: normalized=%.6f -> raw=%u (0x%02X), torque=%d, velocity=%d",
                          i, normalized, raw_positions[i], raw_positions[i], torques[i], velocities[i]);
            }

            // Summary log: all raw positions being sent
            std::string raw_str = "Raw positions [";
            for (size_t i = 0; i < JOINT_COUNT; ++i)
            {
                if (i > 0) raw_str += ", ";
                raw_str += std::to_string(raw_positions[i]);
            }
            raw_str += "]";
            RCLCPP_INFO(logger_, "LinkerHand %s: Sending Modbus FC16 command to slave 0x%02X, start_addr=0x%04X, %zu registers: %s", 
                       Traits::PRODUCT_NAME, slave_id_, 
                       ModbusConfig::LinkerHand::THUMB_PITCH_REG, JOINT_COUNT, raw_str.c_str());

            // Use Modbus function code 16 (Write Multiple Holding Registers) to write all positions at once
            // Start address: 0x0000 (THUMB_PITCH_REG), count: JOINT_COUNT
            return writeMultipleRegisters(slave_id_, 
                                         ModbusConfig::LinkerHand::THUMB_PITCH_REG, 
                                         raw_positions,
                                         ModbusConfig::LinkerHand::WRITE_FUNCTION);
        }

        bool getStatus() override
        {
            // Read position registers (JOINT_COUNT registers)
            return sendReadRequestAsync(slave_id_, 
                                       ModbusConfig::LinkerHand::THUMB_PITCH_REG, 
                                       static_cast<uint16_t>(JOINT_COUNT), 
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
            RCLCPP_INFO(logger_, "LinkerHand %s (slave 0x%02X): Received %zu bytes: %s...",
                        Traits::PRODUCT_NAME, slave_id_, data_size, hex_str.c_str());
            
            // Parse Modbus response using inherited parseModbusResponse
            std::vector<uint16_t> registers = parseModbusResponse(
                data, data_size, 
                slave_id_, 
                ModbusConfig::LinkerHand::READ_FUNCTION
            );
            
            // Check if we got enough registers
            if (registers.size() < JOINT_COUNT)
            {
                RCLCPP_WARN(logger_, "LinkerHand %s: Expected %zu registers, got %zu (data_size=%zu, slave=0x%02X, func=0x%02X)",
                           Traits::PRODUCT_NAME, JOINT_COUNT, registers.size(), data_size, 
                           slave_id_, ModbusConfig::LinkerHand::READ_FUNCTION);
                if (data_size >= 2)
                {
                    RCLCPP_WARN(logger_, "  Actual response: slave=0x%02X, func=0x%02X, byte_count=%u",
                               data[0], data[1], data_size >= 3 ? data[2] : 0);
                }
                return false;
            }
            
            // Convert register values to normalized positions [0.0, 1.0]
            // Protocol: 0 = bend/close, 255 = straight/open
            // So normalized = (255 - raw) / 255.0
            positions.reserve(JOINT_COUNT);
            for (size_t i = 0; i < JOINT_COUNT; ++i)
            {
                // Each register contains one byte (0-255) in the low byte
                uint8_t raw_pos = static_cast<uint8_t>(registers[i] & 0xFF);
                // Convert to normalized [0.0, 1.0]: 0 (bend) -> 1.0, 255 (straight) -> 0.0
                double normalized = (255.0 - static_cast<double>(raw_pos)) / 255.0;
                normalized = std::max(0.0, std::min(1.0, normalized));
                positions.push_back(normalized);
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
                if (name_lower.find("thumb_joint3") != std::string::npos) return 2;
                if (name_lower.find("index_joint") != std::string::npos)  return 3;
                if (name_lower.find("middle_joint") != std::string::npos) return 4;
                if (name_lower.find("ring_joint") != std::string::npos)   return 5;
                if (name_lower.find("pinky_joint") != std::string::npos)  return 6;
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
            // Reset hand state if needed
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
