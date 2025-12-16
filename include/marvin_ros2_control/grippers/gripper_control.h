#pragma once

#include <vector>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>

namespace marvin_ros2_control
{
    // Standard Modbus function codes (for reference)
    namespace modbus_functions
    {
        static constexpr uint8_t READ_HOLDING_REGISTERS = 0x03;
        static constexpr uint8_t READ_INPUT_REGISTERS = 0x04;
        static constexpr uint8_t WRITE_SINGLE_REGISTER = 0x06;
        static constexpr uint8_t WRITE_MULTIPLE_REGISTERS = 0x10;
    }

    // Modbus function types
    using Clear485Func = bool (*)();
    using Send485Func = bool (*)(uint8_t data[256], long size, long channel);

    class ModbusIO
    {
    public:
        // Constructor with 485 function pointers
        ModbusIO(Clear485Func clear_485, Send485Func send_485);

        // Logger initialization
        static void initLogger(const rclcpp::Logger& logger) { logger_ = logger; }

        // Generic Modbus operations
        std::vector<uint16_t> readRegisters(uint8_t slave_id, uint16_t start_addr,
                                            uint16_t count, uint8_t function_code);

        bool writeSingleRegister(uint8_t slave_id, uint16_t register_addr, uint16_t value,
                                 uint8_t function_code);

        bool writeMultipleRegisters(uint8_t slave_id, uint16_t start_addr,
                                    const std::vector<uint16_t>& values,
                                    uint8_t function_code);

        // Response handling
        std::vector<uint8_t> receiveResponse(int max_attempts = 20, int timeout_ms = 50);

    private:
        // Core communication
        bool sendRequest(const std::vector<uint8_t>& request);

        // Request building
        std::vector<uint8_t> buildRequest(uint8_t slave_id, uint8_t function_code,
                                          const std::vector<uint8_t>& data);

        // CRC calculation
        uint16_t calculateCRC(const uint8_t* data, size_t length);

        // Member variables for 485 functions
        Clear485Func clear_485_;
        Send485Func send_485_;

        // Static logger instance
        static rclcpp::Logger logger_;

        // Constants
        static constexpr uint8_t COM1_CHANNEL = 2;
        static constexpr size_t MAX_MODBUS_REGISTERS = 125;
        static constexpr size_t MAX_BUFFER_SIZE = 256;
    };

    // ==============================================
    // Abstract Base Gripper Class
    // ==============================================

    class ModbusGripper
    {
    public:
        virtual ~ModbusGripper() = default;

        // Logger initialization (static)
        static void initLogger(const rclcpp::Logger& logger)
        {
            logger_ = logger;
            ModbusIO::initLogger(logger);
        }

        // Pure virtual functions that must be implemented by derived classes
        virtual bool initialize() = 0;
        virtual bool move_gripper(int torque, int velocity, int position) = 0;
        virtual bool getStatus(int& torque, int& velocity, int& position) = 0;

        virtual void deinitialize()
        {
        };

        // Common utility functions
        virtual void resetState()
        {
            /* Can be overridden if needed */
        }

    protected:
        // Protected constructor for derived classes
        ModbusGripper(Clear485Func clear_485, Send485Func send_485)
            : modbus_io_(clear_485, send_485)
        {
        }

        // Modbus IO instance accessible by derived classes
        ModbusIO modbus_io_;

        // Protected helper methods for derived classes
        bool writeSingleRegister(uint8_t slave_id, uint16_t register_addr, uint16_t value,
                                 uint8_t function_code)
        {
            return modbus_io_.writeSingleRegister(slave_id, register_addr, value, function_code);
        }

        bool writeMultipleRegisters(uint8_t slave_id, uint16_t start_addr,
                                    const std::vector<uint16_t>& values,
                                    uint8_t function_code)
        {
            return modbus_io_.writeMultipleRegisters(slave_id, start_addr, values, function_code);
        }

        std::vector<uint16_t> readRegisters(uint8_t slave_id, uint16_t start_addr, uint16_t count,
                                            uint8_t function_code)
        {
            return modbus_io_.readRegisters(slave_id, start_addr, count, function_code);
        }

        // Static logger accessible by derived classes
        static rclcpp::Logger logger_;
    };

    // ==============================================
    // Specific Gripper Implementations (Forward Declarations)
    // ==============================================
    // Implementations are in separate files:
    // - marvin_ros2_control/grippers/changingtek_gripper.h
    // - marvin_ros2_control/grippers/jd_gripper.h

    class ChangingtekGripper;
    class JDGripper;
}
