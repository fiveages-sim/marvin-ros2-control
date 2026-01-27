#pragma once

#include <vector>
#include <cstdint>
#include <cstddef>
#include "rclcpp/logger.hpp"

// Forward declaration - MarvinSDK.h will be included in implementation files
extern "C" {
    long OnGetChDataA(unsigned char*, long*);
}

namespace marvin_ros2_control
{
    // Forward declarations
    using Clear485Func = bool(*)();
    using Send485Func = bool(*)(uint8_t*, long, long);
    using GetChDataFunc = long(*)(unsigned char*, long*);

    // Constants
    constexpr long COM1_CHANNEL = 2;
    constexpr size_t MAX_MODBUS_REGISTERS = 125;
    constexpr size_t MAX_BUFFER_SIZE = 256;

    /**
     * @brief Modbus IO interface for Marvin robots
     * 
     * Provides low-level Modbus communication using Marvin SDK functions.
     * This class is specific to Marvin robots and cannot be shared with other packages.
     */
    class ModbusIO
    {
    public:
        ModbusIO(Clear485Func clear_485, Send485Func send_485,
                 GetChDataFunc on_get_ch_data = nullptr);
        
        virtual ~ModbusIO() = default;

        // Modbus communication methods
        std::vector<uint16_t> readRegisters(uint8_t slave_id, uint16_t start_addr,
                                           uint16_t count, uint8_t function_code);
        
        bool sendReadRequestAsync(uint8_t slave_id, uint16_t start_addr, 
                                 uint16_t count, uint8_t function_code);
        
        std::vector<uint16_t> parseModbusResponse(const uint8_t* data, size_t data_size,
                                                  uint8_t expected_slave_id, 
                                                  uint8_t expected_function_code);
        
        bool writeSingleRegister(uint8_t slave_id, uint16_t register_addr, uint16_t value,
                                uint8_t function_code);
        
        bool writeMultipleRegisters(uint8_t slave_id, uint16_t start_addr,
                                   const std::vector<uint16_t>& values,
                                   uint8_t function_code);

    protected:
        // Protected methods for derived classes
        bool sendRequest(const std::vector<uint8_t>& request);
        std::vector<uint8_t> receiveResponse(int max_attempts, int timeout_ms);
        std::vector<uint8_t> buildRequest(uint8_t slave_id, uint8_t function_code,
                                         const std::vector<uint8_t>& data);
        uint16_t calculateCRC(const uint8_t* data, size_t length);

        // Protected members
        Clear485Func clear_485_;
        Send485Func send_485_;
        GetChDataFunc on_get_ch_data_;
        
        // Static logger (initialized in .cpp)
        static rclcpp::Logger logger_;
    };
} // namespace marvin_ros2_control

