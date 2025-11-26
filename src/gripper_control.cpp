#include "gripper_control.h"
#include "MarvinSDK.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>

namespace tj2_ros2_control
{
    bool ZXGripper::acc_set = false;
    bool ZXGripper::deacc_set = false;

    std::vector<uint16_t> ModbusIO::readHoldingRegisters(uint8_t slaveId, uint16_t startAddr, uint16_t count) {
        if (count > 125) count = 125;
        
        std::vector<uint8_t> data = {
            (uint8_t)(startAddr >> 8), (uint8_t)(startAddr & 0xFF),
            (uint8_t)(count >> 8), (uint8_t)(count & 0xFF)
        };
        
        auto request = buildRequest(slaveId, 0x03, data);
        if (!sendRequest(request)) return {};
        
        auto response = receiveResponse();
        if (response.size() < 5) return {};
        
        // Parse response - FIXED: Proper Modbus response parsing
        std::vector<uint16_t> result;
        uint8_t byteCount = response[2];
        
        // Modbus response format: [slaveId][funcCode][byteCount][data...][crcLow][crcHigh]
        for (size_t i = 3; i < 3 + byteCount; i += 2) {
            if (i + 1 < response.size()) {
                uint16_t value = (response[i] << 8) | response[i + 1];  // Big-endian
                result.push_back(value);
            }
        }
        
        return result;
    }

    std::vector<uint16_t> ModbusIO::readInputRegisters(uint8_t slaveId, uint16_t startAddr, uint16_t count) {
        if (count > 125) count = 125;
        
        std::vector<uint8_t> data = {
            (uint8_t)(startAddr >> 8), (uint8_t)(startAddr & 0xFF),
            (uint8_t)(count >> 8), (uint8_t)(count & 0xFF)
        };
        
        auto request = buildRequest(slaveId, 0x04, data);
        if (!sendRequest(request)) return {};
        
        auto response = receiveResponse();
        if (response.size() < 5) return {};
        
        std::vector<uint16_t> result;
        uint8_t byteCount = response[2];
        
        for (size_t i = 3; i < 3 + byteCount; i += 2) {
            if (i + 1 < response.size()) {
                uint16_t value = (response[i] << 8) | response[i + 1];
                result.push_back(value);
            }
        }
        
        return result;
    }

    bool ModbusIO::writeSingleRegister(uint8_t slaveId, uint16_t registerAddr, uint16_t value) {
        std::vector<uint8_t> data = {
            (uint8_t)(registerAddr >> 8), (uint8_t)(registerAddr & 0xFF),
            (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)
        };
        
        auto request = buildRequest(slaveId, 0x06, data);
        if (!sendRequest(request)) return false;
        
        auto response = receiveResponse();
        
        // Verify response matches request (echo)
        if (response.size() >= 8) {
            // Check if response echoes our request
            return (response[0] == slaveId && response[1] == 0x06);
        }
        return false;
    }

    bool ModbusIO::writeMultipleRegisters(uint8_t slaveId, uint16_t startAddr, const std::vector<uint16_t>& values, std::vector<uint8_t>& response) {
        if (values.empty() || values.size() > 123) return false;
        
        std::vector<uint8_t> data = {
            (uint8_t)(startAddr >> 8), (uint8_t)(startAddr & 0xFF),
            (uint8_t)(values.size() >> 8), (uint8_t)(values.size() & 0xFF),
            (uint8_t)(values.size() * 2)
        };
        
        for (uint16_t value : values) {
            data.push_back((uint8_t)(value >> 8));
            data.push_back((uint8_t)(value & 0xFF));
        }
        
        auto request = buildRequest(slaveId, 0x10, data);
        if (!sendRequest(request)) return false;
        
        response = receiveResponse();
        return response.size() >= 8;
    }

    bool ModbusIO::sendRequest(std::vector<uint8_t>& request) {
        // Debug output
        char debug_str[512];
        hex_to_str(request.data(), request.size(), debug_str, sizeof(debug_str));
        std::cout << "Sending Modbus request: " << debug_str << std::endl;
        
        return OnSetChDataA(request.data(), request.size(), 2); // COM1
    }

    std::vector<uint8_t> ModbusIO::receiveResponse() {
        unsigned char buffer[256];
        long channel = 2; // COM1
        
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // Wait for response with timeout
        for (int i = 0; i < 20; i++) {  // Increased attempts for better reliability
            int received = OnGetChDataA(buffer, &channel);
            if (received > 0) {
                std::vector<uint8_t> response(buffer, buffer + received);
                
                // Debug output
                char debug_str[512];
                hex_to_str(response.data(), response.size(), debug_str, sizeof(debug_str));
                std::cout << "Received Modbus response: " << debug_str << std::endl;
                
                return response;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        
        std::cout << "Modbus response timeout!" << std::endl;
        return {};
    }

    std::vector<uint8_t> ModbusIO::buildRequest(uint8_t slaveId, uint8_t functionCode, const std::vector<uint8_t>& data) {
        std::vector<uint8_t> request;
        request.push_back(slaveId);
        request.push_back(functionCode);
        request.insert(request.end(), data.begin(), data.end());
        
        uint16_t crc = calculateCRC(request.data(), request.size());
        request.push_back(crc & 0xFF);
        request.push_back(crc >> 8);
        
        return request;
    }

    uint16_t ModbusIO::calculateCRC(const uint8_t* data, size_t length) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < length; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 0x0001) {
                    crc = (crc >> 1) ^ 0xA001;
                } else {
                    crc = crc >> 1;
                }
            }
        }
        return crc;
    }

    bool ZXGripper::ZXGripperInit() {
        // write 01 06 01 00 00 01 49 F6
        // return 01 06 01 00 00 01 49 F6
        uint8_t slave_id = 0x01;
        uint16_t slave_add = 0x0100;
        uint16_t value = 0x0001;
        ZXGripper::deacc_set = false;
        ZXGripper::acc_set = false;
        bool success = ModbusIO::writeSingleRegister(slave_id, slave_add, value);
        std::cout << "Gripper initialization: " << (success ? "SUCCESS" : "FAILED") << std::endl;
        return success;
    }

    bool ZXGripper::ZXGripperMove(int& trq_set, int& vel_set, int& pos_set) {
        // 01 10 01 02 00 02 04 00 00 00 64 7E 0D 位置
        // 01 06 01 04 00 64 C8 1C 速度
        // 01 06 01 06 07 D0 6B 9B 加速度
        // 01 06 01 05 00 64 99 DC 力矩
        // 01 06 01 07 07 D0 3A 5B 减速
        // 01 06 01 08 00 01 C8 34 运动触发

        uint8_t slave_id = 0x01;
        uint16_t vel = static_cast<uint16_t>(vel_set & 0xFFFF);
        uint16_t trq = static_cast<uint16_t>(trq_set & 0xFFFF);
        uint16_t acc = 2000;
        uint16_t deacc = 2000;
        uint16_t trigger = 0x0001;
        
        std::vector<uint16_t> pos_vec = {0x0000, static_cast<uint16_t>(pos_set & 0xFFFF)};
        
        std::vector<uint8_t> response;
        bool result = true;
        
        // Write position (multiple registers)
        result = result && ModbusIO::writeMultipleRegisters(slave_id, 0x0102, pos_vec, response);
        std::cout << "Write position: " << (result ? "SUCCESS" : "FAILED") << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        // Write velocity
        result = result && ModbusIO::writeSingleRegister(slave_id, 0x0104, vel);
        std::cout << "Write velocity: " << (result ? "SUCCESS" : "FAILED") << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        
        // Write torque
        result = result && ModbusIO::writeSingleRegister(slave_id, 0x0105, trq);
        std::cout << "Write torque: " << (result ? "SUCCESS" : "FAILED") << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        
        if (!ZXGripper::acc_set)
        {
            // Write acceleration
            result = result && ModbusIO::writeSingleRegister(slave_id, 0x0106, acc);
            std::cout << "Write acceleration: " << (result ? "SUCCESS" : "FAILED") << std::endl;
            ZXGripper::acc_set = true;
        }

        if (!ZXGripper::deacc_set)
        {
            // Write deceleration
            result = result && ModbusIO::writeSingleRegister(slave_id, 0x0107, deacc);
            std::cout << "Write deceleration: " << (result ? "SUCCESS" : "FAILED") << std::endl;
            ZXGripper::deacc_set = true;
        }
        
        // Write trigger
        result = result && ModbusIO::writeSingleRegister(slave_id, 0x0108, trigger);
        std::cout << "Write trigger: " << (result ? "SUCCESS" : "FAILED") << std::endl;
        
        return result;
    }

    bool ZXGripper::ZXGripperStatus(int& trq_set, int& vel_set, int& pos_set) {
        // 01 03 06 09 00 02
        uint8_t slave_id = 0x01;
        uint16_t slave_add = 0x060D;
        uint16_t count = 0x0002;
        
        std::vector<uint16_t> response = ModbusIO::readHoldingRegisters(slave_id, slave_add, count);
        
        bool success = response.size() >= 2;
        if (success) {
            // Assuming response[0] is velocity and response[1] is position
            // Adjust based on actual Modbus register mapping
            pos_set = response[0] << 16 | response[1];  // Combine two 16-bit registers into 32-bit position
            vel_set = 0;  // If you have velocity data, set it here
            trq_set = 0;  // If you have torque data, set it here
        }
        
        std::cout << "Gripper status - Position: " << pos_set 
                  << ", Raw data: " << (response.size() > 0 ? std::to_string(response[0]) : "N/A") 
                  << ", " << (response.size() > 1 ? std::to_string(response[1]) : "N/A")
                  << ", Success: " << success << std::endl;
        
        return success;
    }

    void ZXGripper::ZXGripperDeInit() {
        // Clean up if needed
        // Note: Marvin SDK cleanup should be handled by the main application
    }
}