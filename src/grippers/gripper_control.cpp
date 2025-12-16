#include "marvin_ros2_control/grippers/gripper_control.h"
#include "MarvinSDK.h"
#include <thread>
#include <chrono>

namespace marvin_ros2_control
{
    // Initialize static logger
    rclcpp::Logger ModbusIO::logger_ = rclcpp::get_logger("modbus_io");
    rclcpp::Logger ModbusGripper::logger_ = rclcpp::get_logger("modbus_gripper");

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

    ModbusIO::ModbusIO(Clear485Func clear_485, Send485Func send_485)
        : clear_485_(clear_485), send_485_(send_485)
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
        std::vector<uint16_t> result = {};
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
        RCLCPP_DEBUG(logger_, "Sending: %s", debug_str);

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
