#ifndef MODBUS_IO_H
#define MODBUS_IO_H

#include <vector>
#include <cstdint>
#include <string>

namespace tj2_ros2_control
{
    // 将十六进制数据转换为字符串
    inline void hex_to_str(const unsigned char* data, int size, char* output, int output_size) {
        int pos = 0;
        for (int i = 0; i < size && pos < output_size - 3; i++) {
            // 每个字节转换为两个十六进制字符
            sprintf(output + pos, "%02X ", data[i]);
            pos += 3;
        }
        if (pos > 0) {
            output[pos - 1] = '\0'; // 替换最后一个空格为结束符
        } else {
            output[0] = '\0';
        }
    }

    // 将十六进制字符串转换为字节数组
    inline int hex_string_to_bytes(const char* hex_str, unsigned char* bytes, int max_bytes) {
        int count = 0;
        char byte_str[3] = {0};
        const char* pos = hex_str;

        while (*pos && count < max_bytes) {
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

class ModbusIO {
public:
    // 读取保持寄存器
    static std::vector<uint16_t> readHoldingRegisters(uint8_t slaveId, uint16_t startAddr, uint16_t count);
    
    // 读取输入寄存器  
    static std::vector<uint16_t> readInputRegisters(uint8_t slaveId, uint16_t startAddr, uint16_t count);
    
    // 写入单个寄存器
    static bool writeSingleRegister(uint8_t slaveId, uint16_t registerAddr, uint16_t value);
    
    // 写入多个寄存器
    static bool writeMultipleRegisters(uint8_t slaveId, uint16_t startAddr, const std::vector<uint16_t>& values, std::vector<uint8_t>& response);

private:
    static bool sendRequest(std::vector<uint8_t>& request);  // Keep as const
    static std::vector<uint8_t> receiveResponse();
    static std::vector<uint8_t> buildRequest(uint8_t slaveId, uint8_t functionCode, const std::vector<uint8_t>& data);
    static uint16_t calculateCRC(const uint8_t* data, size_t length);
};

class ZXGripper
{
    public:
        static bool ZXGripperMove(int& trq_set, int& vel_set, int& pos_set);
        static bool ZXGripperStatus(int& trq_set, int& vel_set, int& pos_set);
        static bool ZXGripperInit();
        static void ZXGripperDeInit();
};       
}

#endif