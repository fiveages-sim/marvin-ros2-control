#include "marvin_ros2_control/grippers/jd_gripper.h"
#include "MarvinSDK.h"
#include <thread>
#include <chrono>
#include <algorithm>
#include "gripper_hardware_common/utils/PositionConverter.h"
#include "gripper_hardware_common/utils/ModbusConfig.h"
#include "gripper_hardware_common/utils/JodellCommandBuilder.h"

namespace marvin_ros2_control
{
    JDGripper::JDGripper(Clear485Func clear_485, Send485Func send_485,
                         GetChDataFunc on_get_ch_data)
        : ModbusGripper(clear_485, send_485, on_get_ch_data)
    {
    }

    // 清除并设置 rACT=0: 09 10 03 E8 00 01 02 00 00 [CRC]
    // 写入寄存器 0x03E8，值为 0x0000
    bool JDGripper::initialize()
    {
        using namespace gripper_hardware_common::ModbusConfig;
        RCLCPP_INFO(logger_, "Initializing JD Gripper (slave: 0x%02X)", Jodell::SLAVE_ADDRESS);
        return writeMultipleRegisters(Jodell::SLAVE_ADDRESS, Jodell::INIT_REG_ADDR, 
                                     {0x0000}, Jodell::WRITE_FUNCTION);
    }

    /// input torque is uint8_t
    /// input velocity is uint8_t
    /// input position is normalized (0.0=closed, 1.0=open)
    /// 
    /// Modbus 协议格式（根据 JD Gripper 协议文档）:
    /// 请求：09 10 03 E8 00 03 06 00 09 [POS] [00] [FORCE] [VEL] [CRC]
    /// - 09: 从站地址
    /// - 10: 功能代码 16（写入多个寄存器）
    /// - 03E8: 请求写入的第一个寄存器的地址
    /// - 0003: 请求写入的寄存器的数量（3个寄存器）
    /// - 06: 数据字节数（3个寄存器 × 2个字节 = 6个字节）
    /// - 0009: 写入寄存器 03E8 的内容(激活请求：09)
    /// - [POS]00: 写入寄存器 03E9 的内容(位置：高字节为位置值，低字节为0x00)
    /// - [FORCE][VEL]: 写入寄存器 03EA 的内容(速度：低字节，力：高字节)
    bool JDGripper::move_gripper(int trq_set, int vel_set, double normalized_pos)
    {
        using namespace gripper_hardware_common;
        using namespace gripper_hardware_common::ModbusConfig;
        
        // 协议: 09 10 03 E8 00 03 06 00 09 [POS] 00 FF FF [CRC]
        // 寄存器 03E8: 0x0009 (激活请求)
        // 寄存器 03E9: [POS]00 (位置值在高字节，低字节为0x00)
        // 寄存器 03EA: 0xFFFF (速度：低字节 FF，力：高字节 FF)
        int pos_set = PositionConverter::Jodell::normalizedToJodell(normalized_pos);
        std::cout << "++++++++++ Poistion set value is :  " << pos_set << std::endl;
        pos_set = std::max(0, std::min(255, pos_set));
        
        std::vector<uint16_t> command_values = {
            0x0009,                                    // 寄存器 03E8: 激活请求
            static_cast<uint16_t>(pos_set << 8),       // 寄存器 03E9: 位置值 (高字节)
            0xFFFF                                     // 寄存器 03EA: 力 0xFF + 速度 0xFF
        };
        
        // 写入多个寄存器：从地址 0x03E8 开始，写入 3 个寄存器
        // 功能代码 0x10 (写入多个寄存器)
        bool result = writeMultipleRegisters(
            Jodell::SLAVE_ADDRESS,      // 从站地址 0x09
            Jodell::POSITION_REG_ADDR,  // 起始寄存器地址 0x03E8
            command_values,
            Jodell::WRITE_FUNCTION      // 功能代码 0x10
        );
        
        if (!result)
        {
            RCLCPP_ERROR(logger_, "Failed to send move command to JD Gripper");
        }
        
        return result;
    }

    bool JDGripper::getStatus()
    {
        using namespace gripper_hardware_common;
        using namespace gripper_hardware_common::ModbusConfig;
        
        // Only send read request, don't wait for response
        // The actual status will be updated by recv_thread_func when response arrives
        return sendReadRequestAsync(Jodell::SLAVE_ADDRESS, Jodell::STATUS_REG_ADDR, 
                                    Jodell::STATUS_REG_COUNT, Jodell::READ_FUNCTION);
    }

    // 响应协议: 寄存器格式
    // 寄存器 07D0: 电动夹爪状态（低字节）
    // 寄存器 07D1: 当前位置（高字节），故障状态（低字节）
    // 寄存器 07D2: 当前力矩（高字节），当前速度（低字节）
    void JDGripper::updateStatusFromResponse(const std::vector<uint16_t>& registers)
    {
        using namespace gripper_hardware_common;
        using namespace gripper_hardware_common::ModbusConfig;
        
        if (registers.size() >= Jodell::STATUS_REG_COUNT)
        {
            // 打印读取到的寄存器值
            RCLCPP_INFO(logger_, "📥 JD Gripper Status Response:");
            RCLCPP_INFO(logger_, "  Register[0] (07D0): 0x%04X - 电动夹爪状态: 0x%02X", 
                       registers[0], registers[0] & 0xFF);
            RCLCPP_INFO(logger_, "  Register[1] (07D1): 0x%04X - 当前位置: 0x%02X, 故障状态: 0x%02X", 
                       registers[1], (registers[1] >> 8) & 0xFF, registers[1] & 0xFF);
            RCLCPP_INFO(logger_, "  Register[2] (07D2): 0x%04X - 当前力矩: 0x%02X, 当前速度: 0x%02X", 
                       registers[2], (registers[2] >> 8) & 0xFF, registers[2] & 0xFF);
            
            // 根据协议提取数据：
            // 寄存器 07D1: 当前位置在高字节
            int position_raw = static_cast<int>(registers[1] >> 8);
            cached_position_ = PositionConverter::Jodell::jodellToNormalized(position_raw);
            // 寄存器 07D2: 当前速度在低字节
            cached_velocity_ = static_cast<int>(registers[2] & 0xFF);
            // 寄存器 07D2: 当前力矩在高字节
            cached_torque_ = static_cast<int>(registers[2] >> 8);
            
            RCLCPP_INFO(logger_, "  Parsed: Position=%d (normalized=%.3f), Velocity=%d, Torque=%d",
                       position_raw, cached_position_, cached_velocity_, cached_torque_);
            
            status_valid_ = true;
        }
    }

    // 响应协议: 09 04 06 00 F1 FE 04 01 00 0B 3F
    // 09: 从站号
    // 04: 功能代码 04（读取输入寄存器）
    // 06: 数据字节数（3个寄存器 × 2个字节 = 6个字节）
    // 00F1: 寄存器 07D0 的内容（电动夹爪状态：F1在低字节）
    // FE04: 寄存器 07D1 的内容（故障状态：04在低字节，当前位置：FE在高字节）
    // 0100: 寄存器 07D2 的内容（当前速度：00在低字节，当前力矩：01在高字节）
    // 0B3F: CRC 校验
    bool JDGripper::processReadResponse(const uint8_t* data, size_t data_size,
                                       int& torque, int& velocity, double& position)
    {
        using namespace gripper_hardware_common;
        using namespace gripper_hardware_common::ModbusConfig;
        
        if (data_size < 3)
        {
            return false;
        }
        
        // 打印所有原始读取数据，不过滤
        std::string hex_str;
        for (size_t i = 0; i < data_size; ++i)
        {
            char hex[4];
            snprintf(hex, sizeof(hex), "%02X ", data[i]);
            hex_str += hex;
        }
        RCLCPP_INFO(logger_, "📥 JD Gripper Raw Response (%zu bytes): %s", data_size, hex_str.c_str());
        
        // 不验证，直接尝试解析所有数据
        // 如果 slave_id 或 function_code 不匹配，仍然打印信息但继续处理
        if (data[0] != SLAVE_ID || data[1] != READ_FUNCTION)
        {
            RCLCPP_INFO(logger_, "⚠️ Response mismatch: slave=0x%02X (expected 0x%02X), func=0x%02X (expected 0x%02X) - but will try to parse anyway",
                       data[0], SLAVE_ID, data[1], READ_FUNCTION);
        }
        
        // 尝试解析，即使验证失败也继续
        std::vector<uint16_t> registers;
        if (data_size >= 5)
        {
            uint8_t byte_count = data[2];
            if (data_size >= static_cast<size_t>(3 + byte_count + 2))
            {
                size_t reg_count = byte_count / 2;
                registers.reserve(reg_count);
                for (size_t i = 0; i < reg_count; ++i)
                {
                    size_t idx = 3 + i * 2;
                    uint16_t value = static_cast<uint16_t>(data[idx] << 8) | data[idx + 1];
                    registers.push_back(value);
                }
            }
        }
        
        if (registers.size() < Jodell::STATUS_REG_COUNT)
        {
            RCLCPP_WARN(logger_, "⚠️ Insufficient registers: got %zu, expected %d - but will try to parse available data", 
                       registers.size(), Jodell::STATUS_REG_COUNT);
            if (registers.empty())
            {
                return false;
            }
        }
        
        // 打印解析后的寄存器值
        RCLCPP_INFO(logger_, "📥 JD Gripper Parsed Registers:");
        RCLCPP_INFO(logger_, "  Register[0] (07D0): 0x%04X - 电动夹爪状态: 0x%02X", 
                   registers[0], registers[0] & 0xFF);
        RCLCPP_INFO(logger_, "  Register[1] (07D1): 0x%04X - 当前位置: 0x%02X, 故障状态: 0x%02X", 
                   registers[1], (registers[1] >> 8) & 0xFF, registers[1] & 0xFF);
        RCLCPP_INFO(logger_, "  Register[2] (07D2): 0x%04X - 当前力矩: 0x%02X, 当前速度: 0x%02X", 
                   registers[2], (registers[2] >> 8) & 0xFF, registers[2] & 0xFF);
        
        // 根据协议提取数据：
        // 寄存器 07D1: 当前位置在高字节 (registers[1] >> 8)
        // 寄存器 07D2: 当前速度在低字节 (registers[2] & 0xFF)
        // 寄存器 07D2: 当前力矩在高字节 (registers[2] >> 8)
        int position_raw = static_cast<int>(registers[1] >> 8);  // 高字节：当前位置
        position = PositionConverter::Jodell::jodellToNormalized(position_raw);
        velocity = static_cast<int>(registers[2] & 0xFF);         // 低字节：当前速度
        torque = static_cast<int>(registers[2] >> 8);            // 高字节：当前力矩
        
        RCLCPP_INFO(logger_, "  Parsed: Position=%d (normalized=%.3f), Velocity=%d, Torque=%d",
                   position_raw, position, velocity, torque);
        
        return true;
    }
} // namespace marvin_ros2_control

