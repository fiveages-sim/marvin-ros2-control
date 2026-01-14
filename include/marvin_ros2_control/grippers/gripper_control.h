#pragma once

#include <vector>
#include <atomic>

// ROS2 includes
#include <rclcpp/rclcpp.hpp>

// Common gripper interface
#include "gripper_hardware_common/GripperBase.h"

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
    using GetChDataFunc = long (*)(uint8_t data[256], long* channel);

    class ModbusIO
    {
    public:
        // Constructor with 485 function pointers and data receive callbacks
        ModbusIO(Clear485Func clear_485, Send485Func send_485,
                 GetChDataFunc on_get_ch_data = nullptr);

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

        // Send read request without waiting for response
        bool sendReadRequestAsync(uint8_t slave_id, uint16_t start_addr, uint16_t count, uint8_t function_code);

        // Parse Modbus RTU response into register values
        static std::vector<uint16_t> parseModbusResponse(const uint8_t* data, size_t data_size, 
                                                         uint8_t expected_slave_id, uint8_t expected_function_code);

        // Get the get_ch_data function pointer
        GetChDataFunc getGetChDataFunc() const { return on_get_ch_data_; }

    protected:
        // Member variables for 485 functions
        GetChDataFunc on_get_ch_data_;

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
    // Abstract Base Hand Class
    // ==============================================

    class ModbusHand : public gripper_hardware_common::GripperBase
    {
    public:
        virtual ~ModbusHand() = default;

        // Logger initialization (static)
        static void initLogger(const rclcpp::Logger& logger)
        {
            logger_ = logger;
            ModbusIO::initLogger(logger);
        }

        // Note: Pure virtual functions (initialize, move_gripper, getStatus) are inherited
        // from gripper_hardware_common::GripperBase and must be implemented by derived classes.
        // getStatus returns normalized position (0.0=closed, 1.0=open).

        // Process received Modbus response (for compatibility with readAndProcessData)
        // Default implementation returns false, should be overridden by derived classes
        virtual bool processReadResponse(const uint8_t* data, size_t data_size, 
                                        int& torque, int& velocity, double& position)
        {
            (void)data;
            (void)data_size;
            (void)torque;
            (void)velocity;
            (void)position;
            return false;
        }

        // Update status from parsed Modbus response
        // Default implementation does nothing, should be overridden by derived classes
        virtual void updateStatusFromResponse(const std::vector<uint16_t>& registers)
        {
            (void)registers;
        }

        // Optional virtual functions with default implementations
        virtual void deinitialize() override
        {
            // Default implementation - can be overridden
        }

        virtual void resetState() override
        {
            // Default implementation - can be overridden
        }

    protected:
        // Protected constructor for derived classes
        ModbusHand(Clear485Func clear_485, Send485Func send_485,
                   GetChDataFunc on_get_ch_data = nullptr)
            : modbus_io_(clear_485, send_485, on_get_ch_data)
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

        // Send read request without waiting (for async operation)
        bool sendReadRequestAsync(uint8_t slave_id, uint16_t start_addr, uint16_t count, uint8_t function_code)
        {
            return modbus_io_.sendReadRequestAsync(slave_id, start_addr, count, function_code);
        }

        // Static logger accessible by derived classes
        static rclcpp::Logger logger_;
    };

    // ==============================================
    // Abstract Base Gripper Class
    // ==============================================

    class ModbusGripper : public gripper_hardware_common::GripperBase
    {
    public:
        virtual ~ModbusGripper() = default;

        // Logger initialization (static)
        static void initLogger(const rclcpp::Logger& logger)
        {
            logger_ = logger;
            ModbusIO::initLogger(logger);
        }

        // Note: Pure virtual functions (initialize, move_gripper, getStatus) are inherited
        // from gripper_hardware_common::GripperBase and must be implemented by derived classes.
        // getStatus returns normalized position (0.0=closed, 1.0=open).

        // Update status from parsed Modbus response (to be implemented by derived classes)
        // Made public so it can be called from recv_thread_func in MarvinHardware
        virtual void updateStatusFromResponse(const std::vector<uint16_t>& registers) = 0;

        // Process received Modbus response: verify, parse, and update status
        // Updates torque, velocity, and position output parameters
        // Returns true if response was successfully processed, false otherwise
        virtual bool processReadResponse(const uint8_t* data, size_t data_size, 
                                        int& torque, int& velocity, double& position) = 0;

        // Read data from port and process it (wrapper function)
        // Returns true if data was successfully read and processed
        bool readAndProcessData(int& torque, int& velocity, double& position)
        {
            GetChDataFunc get_ch_data = modbus_io_.getGetChDataFunc();
            if (!get_ch_data)
            {
                std::cout << "get_ch_data is null" << std::endl;
                return false;
            }
            
            unsigned char data_buf[256] = {0};
            long ch = 2;
            const long size = get_ch_data(data_buf, &ch);
            
            // Always try to process, even if size is 0 (to check for any data)
            if (size > 0)
            {
                return processReadResponse(data_buf, static_cast<size_t>(size), torque, velocity, position);
            }
            // std::cout << "size is 0" << std::endl;
            return false;
        }

        // Get cached status (updated by recv_thread_func)
        bool getCachedStatus(int& torque, int& velocity, double& position) const
        {
            if (status_valid_)
            {
                torque = cached_torque_;
                velocity = cached_velocity_;
                position = cached_position_;
                return true;
            }
            return false;
        }

        // Optional virtual functions with default implementations
        virtual void deinitialize() override
        {
            // Default implementation - can be overridden
        }

        virtual void resetState() override
        {
            // Default implementation - can be overridden
        }

    protected:
        // Protected constructor for derived classes
        ModbusGripper(Clear485Func clear_485, Send485Func send_485,
                      GetChDataFunc on_get_ch_data = nullptr)
            : modbus_io_(clear_485, send_485, on_get_ch_data)
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

        // Send read request without waiting (for async operation)
        bool sendReadRequestAsync(uint8_t slave_id, uint16_t start_addr, uint16_t count, uint8_t function_code)
        {
            return modbus_io_.sendReadRequestAsync(slave_id, start_addr, count, function_code);
        }

        // Static logger accessible by derived classes
        static rclcpp::Logger logger_;

    protected:
        // Status storage (no mutex needed - writes only from recv thread, reads from callback thread)
        int cached_torque_ = 0;
        int cached_velocity_ = 0;
        double cached_position_ = 0.0;
        std::atomic<bool> status_valid_{false};
    };

    // ==============================================
    // Specific Gripper Implementations (Forward Declarations)
    // ==============================================
    // Implementations are in separate files:
    // - marvin_ros2_control/grippers/changingtek_gripper.h
    // - marvin_ros2_control/grippers/jd_gripper.h

    template<typename> class ChangingtekGripper;
    // ChangingtekGripper90C and ChangingtekGripper90D are type aliases defined in changingtek_gripper.h
    class JDGripper;
    class DexterousHandGripper;
}
