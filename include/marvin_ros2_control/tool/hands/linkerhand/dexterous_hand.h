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
                             GetChDataFunc on_get_ch_data = nullptr)
            : ModbusHand(clear_485, send_485, on_get_ch_data)
        {
        }

        bool initialize() override
        {
            RCLCPP_INFO(logger_, "Initializing LinkerHand %s (%zu-DOF, slave: 0x%02X)", 
                       Traits::PRODUCT_NAME, JOINT_COUNT, ModbusConfig::LinkerHand::DEFAULT_SLAVE_ID);
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

        bool move_hand(const std::vector<int>& torques, const std::vector<int>& velocities, const std::vector<double>& positions)
        {
            // Validate input sizes
            if (torques.size() != JOINT_COUNT || velocities.size() != JOINT_COUNT || positions.size() != JOINT_COUNT)
            {
                RCLCPP_ERROR(logger_, "LinkerHand %s: Invalid input size. Expected %zu values, got torques=%zu, velocities=%zu, positions=%zu",
                            Traits::PRODUCT_NAME, JOINT_COUNT, torques.size(), velocities.size(), positions.size());
                return false;
            }

            bool result = true;

            // Use register arrays from ModbusConfig (only use first JOINT_COUNT registers)
            for (size_t i = 0; i < JOINT_COUNT; ++i)
            {
                // Convert normalized position (0.0-1.0) to hand position (0-255)
                uint8_t pos_value = static_cast<uint8_t>(std::max(0.0, std::min(1.0, positions[i])) * 255.0);
                
                // Write position
                result = writeSingleRegister(ModbusConfig::LinkerHand::DEFAULT_SLAVE_ID, 
                                           ModbusConfig::LinkerHand::POSITION_REGS[i], pos_value, 
                                           ModbusConfig::LinkerHand::WRITE_SINGLE_FUNCTION) && result;
                
                // Write torque (0-255 range)
                uint8_t torque_value = static_cast<uint8_t>(std::max(0, std::min(255, torques[i])));
                result = writeSingleRegister(ModbusConfig::LinkerHand::DEFAULT_SLAVE_ID, 
                                           ModbusConfig::LinkerHand::TORQUE_REGS[i], torque_value, 
                                           ModbusConfig::LinkerHand::WRITE_SINGLE_FUNCTION) && result;
                
                // Write speed (0-255 range)
                uint8_t speed_value = static_cast<uint8_t>(std::max(0, std::min(255, velocities[i])));
                result = writeSingleRegister(ModbusConfig::LinkerHand::DEFAULT_SLAVE_ID, 
                                           ModbusConfig::LinkerHand::SPEED_REGS[i], speed_value, 
                                           ModbusConfig::LinkerHand::WRITE_SINGLE_FUNCTION) && result;
            }

            return result;
        }

        bool getStatus() override
        {
            // Read position registers (JOINT_COUNT registers)
            return sendReadRequestAsync(ModbusConfig::LinkerHand::DEFAULT_SLAVE_ID, 
                                       ModbusConfig::LinkerHand::THUMB_PITCH_REG, 
                                       static_cast<uint16_t>(JOINT_COUNT), 
                                       ModbusConfig::LinkerHand::READ_FUNCTION);
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
            return writeSingleRegister(ModbusConfig::LinkerHand::DEFAULT_SLAVE_ID, 
                                      ModbusConfig::LinkerHand::POSITION_REGS[finger_id], position, 
                                      ModbusConfig::LinkerHand::WRITE_SINGLE_FUNCTION);
        }

        bool setFingerTorque(uint8_t finger_id, uint8_t torque)
        {
            if (finger_id >= JOINT_COUNT)
            {
                return false;
            }
            return writeSingleRegister(ModbusConfig::LinkerHand::DEFAULT_SLAVE_ID, 
                                      ModbusConfig::LinkerHand::TORQUE_REGS[finger_id], torque, 
                                      ModbusConfig::LinkerHand::WRITE_SINGLE_FUNCTION);
        }

        bool setFingerSpeed(uint8_t finger_id, uint8_t speed)
        {
            if (finger_id >= JOINT_COUNT)
            {
                return false;
            }
            return writeSingleRegister(ModbusConfig::LinkerHand::DEFAULT_SLAVE_ID, 
                                      ModbusConfig::LinkerHand::SPEED_REGS[finger_id], speed, 
                                      ModbusConfig::LinkerHand::WRITE_SINGLE_FUNCTION);
        }

        bool getFingerPosition(uint8_t finger_id, uint8_t& position)
        {
            if (finger_id >= JOINT_COUNT)
            {
                return false;
            }
            std::vector<uint16_t> response = readRegisters(ModbusConfig::LinkerHand::DEFAULT_SLAVE_ID, 
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
            std::vector<uint16_t> response = readRegisters(ModbusConfig::LinkerHand::DEFAULT_SLAVE_ID, 
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
            std::vector<uint16_t> response = readRegisters(ModbusConfig::LinkerHand::DEFAULT_SLAVE_ID, 
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
