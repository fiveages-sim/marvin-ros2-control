#include "marvin_ros2_control/tool/modbus_io.h"
#include "MarvinSDK.h"
#include "rclcpp/logging.hpp"
#include <thread>
#include <chrono>
#include "rclcpp/logging.hpp"

namespace marvin_ros2_control
{
    // Initialize static loggers
    rclcpp::Logger ModbusIO::logger_ = rclcpp::get_logger("modbus_io");

    inline void hex_to_str(const unsigned char* data, int size, char* output, int output_size)
    {
        int pos = 0;
        for (int i = 0; i < size && pos < output_size - 3; i++)
        {
            // 每个字节转换为两个十六进制字符
            sprintf(output + pos, "%02X ", data[i]);
            pos += 3;
        }
        if (pos > 0)
        {
            output[pos - 1] = '\0'; // 替换最后一个空格为结束符
        }
        else
        {
            output[0] = '\0';
        }
    }

    // 将十六进制字符串转换为字节数组
    inline int hex_string_to_bytes(const char* hex_str, unsigned char* bytes, int max_bytes)
    {
        int count = 0;
        char byte_str[3] = {0};
        const char* pos = hex_str;

        while (*pos && count < max_bytes)
        {
            // 跳过空格
            while (*pos == ' ') pos++;
            if (!*pos) break;

            // 提取两个字符作为一个字节
            byte_str[0] = *pos++;
            if (!*pos) break; // 确保有第二个字符
            byte_str[1] = *pos++;

            // 转换为字节
            bytes[count++] = (unsigned char)strtol(byte_str, NULL, 16);
        }

        return count;
    }

    // ==============================================
    // ModbusIO Implementation
    // ==============================================

    ModbusIO::ModbusIO(Clear485Func clear_485, Send485Func send_485,
                       GetChDataFunc on_get_ch_data)
        : clear_485_(clear_485),
          send_485_(send_485),
          on_get_ch_data_(on_get_ch_data ? on_get_ch_data : OnGetChDataA)
    {

    }

    std::vector<uint16_t> ModbusIO::readRegisters(uint8_t slave_id, uint16_t start_addr,
                                                  uint16_t count, uint8_t function_code)
    {
        if (count > MAX_MODBUS_REGISTERS)
        {
            count = MAX_MODBUS_REGISTERS;
            RCLCPP_WARN(logger_, "Limiting count to %zu", MAX_MODBUS_REGISTERS);
        }

        std::vector<uint8_t> data = {
            static_cast<uint8_t>(start_addr >> 8),
            static_cast<uint8_t>(start_addr & 0xFF),
            static_cast<uint8_t>(count >> 8),
            static_cast<uint8_t>(count & 0xFF)
        };

        auto request = buildRequest(slave_id, function_code, data);

        if (!sendRequest(request))
        {
            RCLCPP_ERROR(logger_, "Failed to send read request");
            return {};
        }

        // Read data using OnGetChDataA until we receive at least one frame (or attempts exhausted)
        std::vector<uint16_t> result;
        uint8_t data_buf[MAX_BUFFER_SIZE];
        char hex_str1[512];
        long set_ch1 = COM1_CHANNEL;

        // Limit attempts to avoid an infinite loop, but preserve the requested read pattern
        const int max_attempts = 50;
        for (int attempt = 0; attempt < max_attempts; ++attempt)
        {
            if (!on_get_ch_data_)
            {
                RCLCPP_ERROR(logger_, "on_get_ch_data_ callback is null");
                return {};
            }

            long tag = on_get_ch_data_(data_buf, &set_ch1);
            std::this_thread::sleep_for(std::chrono::milliseconds(200)); // sleep(0.2) equivalent

            if (tag >= 1)
            {
                hex_to_str(data_buf, static_cast<int>(tag), hex_str1, sizeof(hex_str1));
                RCLCPP_INFO(logger_, "接收信号: %ld, 接收的HEX数据: %s", tag, hex_str1);

                // Basic Modbus RTU parsing: [slave][func][byte_count][data...][crc_low][crc_high]
                std::vector<uint8_t> response(data_buf, data_buf + static_cast<size_t>(tag));
                if (response.size() < 5) // minimal RTU frame size
                {
                    continue;
                }

                // Optional: validate slave id and function code
                if (response[0] != slave_id || response[1] != function_code)
                {
                    continue;
                }

                uint8_t byte_count = response[2];
                // Ensure we have enough data bytes plus CRC
                if (response.size() < static_cast<size_t>(3 + byte_count + 2))
                {
                    continue;
                }

                // Extract register values (big-endian pairs)
                size_t reg_count = byte_count / 2;
                result.reserve(reg_count);
                for (size_t i = 0; i < reg_count; ++i)
                {
                    size_t idx = 3 + i * 2;
                    uint16_t value = static_cast<uint16_t>(response[idx] << 8) | response[idx + 1];
                    result.push_back(value);
                }
                return result;
            }
        }

        RCLCPP_ERROR(logger_, "Modbus read timeout: no valid response after %d attempts", max_attempts);
        return {};
    }

    bool ModbusIO::sendReadRequestAsync(uint8_t slave_id, uint16_t start_addr, uint16_t count, uint8_t function_code)
    {
        if (count > MAX_MODBUS_REGISTERS)
        {
            count = MAX_MODBUS_REGISTERS;
            RCLCPP_WARN(logger_, "Limiting count to %zu", MAX_MODBUS_REGISTERS);
        }

        std::vector<uint8_t> data = {
            static_cast<uint8_t>(start_addr >> 8),
            static_cast<uint8_t>(start_addr & 0xFF),
            static_cast<uint8_t>(count >> 8),
            static_cast<uint8_t>(count & 0xFF)
        };

        auto request = buildRequest(slave_id, function_code, data);
        return sendRequest(request);
    }

    std::vector<uint16_t> ModbusIO::parseModbusResponse(const uint8_t* data, size_t data_size,
                                                        uint8_t expected_slave_id, uint8_t expected_function_code)
    {
        std::vector<uint16_t> result;
        
        if (data_size < 5) // minimal RTU frame size
        {
            RCLCPP_DEBUG(logger_, "parseModbusResponse: Data too short: %zu bytes (min 5)", data_size);
            return result;
        }

        // Validate slave id and function code
        if (data[0] != expected_slave_id || data[1] != expected_function_code)
        {
            RCLCPP_DEBUG(logger_, "parseModbusResponse: Validation failed - expected slave=0x%02X func=0x%02X, got slave=0x%02X func=0x%02X",
                        expected_slave_id, expected_function_code, data[0], data[1]);
            return result;
        }

        uint8_t byte_count = data[2];
        // Ensure we have enough data bytes plus CRC
        if (data_size < static_cast<size_t>(3 + byte_count + 2))
        {
            return result;
        }

        // Extract register values (big-endian pairs)
        size_t reg_count = byte_count / 2;
        result.reserve(reg_count);
        for (size_t i = 0; i < reg_count; ++i)
        {
            size_t idx = 3 + i * 2;
            uint16_t value = static_cast<uint16_t>(data[idx] << 8) | data[idx + 1];
            result.push_back(value);
        }
        
        return result;
    }

    bool ModbusIO::writeSingleRegister(uint8_t slave_id, uint16_t register_addr, uint16_t value,
                                       uint8_t function_code)
    {
        std::vector<uint8_t> data = {
            static_cast<uint8_t>(register_addr >> 8),
            static_cast<uint8_t>(register_addr & 0xFF),
            static_cast<uint8_t>(value >> 8),
            static_cast<uint8_t>(value & 0xFF)
        };

        auto request = buildRequest(slave_id, function_code, data);
        if (!sendRequest(request))
        {
            RCLCPP_ERROR(logger_, "Failed to write register 0x%04X", register_addr);
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        return true;
    }

    bool ModbusIO::writeMultipleRegisters(uint8_t slave_id, uint16_t start_addr,
                                          const std::vector<uint16_t>& values,
                                          uint8_t function_code)
    {
        if (values.empty() || values.size() > MAX_MODBUS_REGISTERS)
        {
            RCLCPP_ERROR(logger_, "Invalid register count: %zu", values.size());
            return false;
        }

        std::vector<uint8_t> data = {
            static_cast<uint8_t>(start_addr >> 8),
            static_cast<uint8_t>(start_addr & 0xFF),
            static_cast<uint8_t>(values.size() >> 8),
            static_cast<uint8_t>(values.size() & 0xFF),
            static_cast<uint8_t>(values.size() * 2)
        };

        for (uint16_t value : values)
        {
            data.push_back(static_cast<uint8_t>(value >> 8));
            data.push_back(static_cast<uint8_t>(value & 0xFF));
        }

        auto request = buildRequest(slave_id, function_code, data);
        if (!sendRequest(request))
        {
            RCLCPP_ERROR(logger_, "Failed to write multiple registers");
            return false;
        }

        return true;
    }

    bool ModbusIO::sendRequest(const std::vector<uint8_t>& request)
    {
        char debug_str[512];
        hex_to_str(request.data(), request.size(), debug_str, sizeof(debug_str));
        RCLCPP_INFO(logger_, "Sending: %s", debug_str);

        return send_485_((uint8_t*)request.data(), static_cast<long>(request.size()), COM1_CHANNEL);
    }

    std::vector<uint8_t> ModbusIO::receiveResponse(int max_attempts, int timeout_ms)
    {
        uint8_t buffer[MAX_BUFFER_SIZE];

        for (int attempt = 0; attempt < max_attempts; attempt++)
        {
            long channel = COM1_CHANNEL;
            int received = OnGetChDataA(buffer, &channel);

            if (received > 0)
            {
                std::cout << "Received gripper message " << received << std::endl;
                std::vector<uint8_t> response(buffer, buffer + received);
                return response;
            }

            if (attempt < max_attempts - 1)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(timeout_ms));
            }
        }

        return {};
    }

    std::vector<uint8_t> ModbusIO::buildRequest(uint8_t slave_id, uint8_t function_code,
                                                const std::vector<uint8_t>& data)
    {
        std::vector<uint8_t> request;
        request.reserve(data.size() + 3);

        request.push_back(slave_id);
        request.push_back(function_code);
        request.insert(request.end(), data.begin(), data.end());

        uint16_t crc = calculateCRC(request.data(), request.size());
        request.push_back(static_cast<uint8_t>(crc & 0xFF));
        request.push_back(static_cast<uint8_t>(crc >> 8));

        return request;
    }

    uint16_t ModbusIO::calculateCRC(const uint8_t* data, size_t length)
    {
        uint16_t crc = 0xFFFF;

        for (size_t i = 0; i < length; i++)
        {
            crc ^= data[i];

            for (int j = 0; j < 8; j++)
            {
                if (crc & 0x0001)
                {
                    crc = (crc >> 1) ^ 0xA001;
                }
                else
                {
                    crc >>= 1;
                }
            }
        }

        return crc;
    }

}
