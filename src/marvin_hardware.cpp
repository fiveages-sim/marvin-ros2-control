#include "marvin_hardware.h"

#include "marvin_ros2_control/sensors/kwr75_marvin485_client.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <random>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cctype>
#include <set>
#include <unordered_map>
#include <cstdint>
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "gripper_hardware_common/ChangingtekGripper.h"
#include "gripper_hardware_common/JodellGripper.h"
#include "marvin_ros2_control/tool/grippers/changingtek/changingtek_gripper.h"
#include "marvin_ros2_control/tool/grippers/jodell/jd_gripper.h"
#include "marvin_ros2_control/tool/grippers/eincinx/eincinx_gripper.h"
#include "marvin_ros2_control/tool/hands/freedom/freedom_hand.h"
#include "marvin_ros2_control/tool/hands/inspire/inspire_hand.h"
#include "marvin_ros2_control/tool/hands/linkerhand/dexterous_hand.h"
#include "marvin_ros2_control/tool/hands/jodell/erg32_hand.h"
#include "marvin_ros2_control/tool/marvin_rs485_bus.h"

using namespace gripper_hardware_common;

namespace marvin_ros2_control
{
    // 辅助函数：标准化字符串（转大写，去除空格）
    static std::string normalizeString(const std::string& str)
    {
        std::string result = str;
        std::transform(result.begin(), result.end(), result.begin(), ::toupper);
        result.erase(std::remove_if(result.begin(), result.end(), ::isspace), result.end());
        return result;
    }

    // 默认参数（单一来源）：同时用于 declare_node_parameters / on_init / applyRobotConfiguration
    static const std::vector<double> kDefaultJointKGains = {2.0, 2.0, 2.0, 1.6, 1.0, 1.0, 1.0};
    static const std::vector<double> kDefaultJointDGains = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
    static const std::vector<double> kDefaultCartKGains  = {1800.0, 1800.0, 1800.0, 40.0, 40.0, 40.0, 20.0};
    static const std::vector<double> kDefaultCartDGains  = {0.6, 0.6, 0.6, 0.4, 0.4, 0.4, 0.4};

    static const std::vector<double> kDefaultLeftKineParam  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static const std::vector<double> kDefaultLeftDynParam   = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static const std::vector<double> kDefaultRightKineParam = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static const std::vector<double> kDefaultRightDynParam  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // ctrl_mode 相关：字符串 <-> 内部 mode(int) 的单一转换来源
    // mode: 1=POSITION, 2=JOINT_IMPEDANCE, 3=CART_IMPEDANCE, 4=POWER_OFF (左右臂均下使能)
    static int ctrlModeStringToMode(const std::string& ctrl_mode_raw, const rclcpp::Logger& logger)
    {
        const std::string normalized = normalizeString(ctrl_mode_raw);
        if (normalized == "POSITION") return 1;
        if (normalized == "JOINT_IMPEDANCE") return 2;
        if (normalized == "CART_IMPEDANCE") return 3;
        if (normalized == "POWER_OFF") return 4;
        RCLCPP_WARN(logger, "Invalid ctrl_mode value: %s, defaulting to POSITION", ctrl_mode_raw.c_str());
        return 1;
    }

    static const char* modeToCtrlModeString(int mode)
    {
        switch (mode) {
            case 1: return "POSITION";
            case 2: return "JOINT_IMPEDANCE";
            case 3: return "CART_IMPEDANCE";
            case 4: return "POWER_OFF";
            default: return "POSITION";
        }
    }

    // 辅助函数：解析 hardware_parameters 中的数组字符串为 vector<double>
    // 支持形如 "[1,2,3]"、"1, 2, 3"、"1 2 3" 等格式
    static std::vector<double> parseDoubleArrayLoose(const std::string& str, const std::vector<double>& default_val)
    {
        if (str.empty()) {
            return default_val;
        }
        std::string cleaned = str;
        cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), '['), cleaned.end());
        cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), ']'), cleaned.end());
        std::replace(cleaned.begin(), cleaned.end(), ',', ' ');

        std::istringstream iss(cleaned);
        std::vector<double> result;
        double v = 0.0;
        while (iss >> v) {
            result.push_back(v);
        }
        return result.empty() ? default_val : result;
    }

    // rclcpp 参数类型映射（用于 declare_node_parameters 的类型检查）
    template<typename T>
    static constexpr rclcpp::ParameterType paramTypeOf();

    template<>
    constexpr rclcpp::ParameterType paramTypeOf<std::string>() { return rclcpp::ParameterType::PARAMETER_STRING; }
    template<>
    constexpr rclcpp::ParameterType paramTypeOf<int>() { return rclcpp::ParameterType::PARAMETER_INTEGER; }
    template<>
    constexpr rclcpp::ParameterType paramTypeOf<double>() { return rclcpp::ParameterType::PARAMETER_DOUBLE; }
    template<>
    constexpr rclcpp::ParameterType paramTypeOf<bool>() { return rclcpp::ParameterType::PARAMETER_BOOL; }
    template<>
    constexpr rclcpp::ParameterType paramTypeOf<std::vector<double>>() { return rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY; }

    static bool parseBoolLoose(const std::string& str, bool default_val)
    {
        const std::string s = normalizeString(str);
        if (s == "TRUE" || s == "1" || s == "YES" || s == "ON") return true;
        if (s == "FALSE" || s == "0" || s == "NO" || s == "OFF") return false;
        return default_val;
    }

    void MarvinHardware::declare_node_parameters()
    {
        // 目标：优先使用 hardware_interface 的 info_.hardware_parameters 作为“初始值来源”，
        // 然后用该初始值声明为 ROS2 node 参数，便于后续 ros2 param set 动态修改。
        //
        // 规则：
        // - 如果参数已存在且类型正确：尊重现有值（可能来自 launch 覆盖/之前声明），不覆盖
        // - 如果参数已存在但类型不对：undeclare 后按正确类型重新 declare（避免类型冲突）
        // - 如果参数不存在：从 hardware_parameters 取值（若有）否则用默认值 declare

        const auto hw_find = [this](const std::string& name) -> const std::string* {
            auto it = info_.hardware_parameters.find(name);
            if (it == info_.hardware_parameters.end()) {
                return nullptr;
            }
            return &it->second;
        };

        const auto ensure_param = [this](const std::string& name, const auto& default_val, const std::string* hw_val, const auto& parse_fn) {
            using T = std::decay_t<decltype(default_val)>;
            if (node_->has_parameter(name)) {
                if (node_->get_parameter(name).get_type() != paramTypeOf<T>()) {
                    try { node_->undeclare_parameter(name); } catch (...) {}
                } else {
                    return; // 已存在且类型正确：尊重现有值
                }
            }

            if (!node_->has_parameter(name)) {
                const T val = hw_val ? parse_fn(*hw_val, default_val) : default_val;
                node_->declare_parameter<T>(name, val);
            }
        };

        // 对 array 参数额外校验长度：类型正确但长度不对也会强制重声明，避免后续运行期兜底/越界风险
        const auto ensure_double_array_sized = [this, &hw_find](const std::string& name,
                                                               const std::vector<double>& default_val,
                                                               size_t expected_size) {
            const std::string* hw_val = hw_find(name);

            if (node_->has_parameter(name)) {
                if (node_->get_parameter(name).get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
                    try { node_->undeclare_parameter(name); } catch (...) {}
                } else {
                    // 类型正确，校验长度
                    try {
                        const auto current = node_->get_parameter(name).get_value<std::vector<double>>();
                        if (current.size() == expected_size) {
                            return; // 已存在且长度正确：尊重现有值
                        }
                    } catch (...) {
                        // fallthrough: undeclare + redeclare
                    }
                    try { node_->undeclare_parameter(name); } catch (...) {}
                }
            }

            if (!node_->has_parameter(name)) {
                std::vector<double> val = hw_val ? parseDoubleArrayLoose(*hw_val, default_val) : default_val;
                if (val.size() != expected_size) {
                    val = default_val;
                }
                node_->declare_parameter<std::vector<double>>(name, val);
            }
        };

        // 基础配置（之前通过 get_param 从 hardware_parameters 读的，也顺便声明成 ROS2 参数）
        ensure_param("arm_type", std::string("LEFT"), hw_find("arm_type"),
                     [](const std::string& s, const std::string& def) { (void)def; return s; });
        ensure_param("left_ee_type", std::string(""), hw_find("left_ee_type"),
                     [](const std::string& s, const std::string& def) { (void)def; return s; });
        ensure_param("right_ee_type", std::string(""), hw_find("right_ee_type"),
                     [](const std::string& s, const std::string& def) { (void)def; return s; });
        ensure_param("left_ee_channel", static_cast<int>(COM1_CHANNEL), hw_find("left_ee_channel"),
                     [](const std::string& s, int def) { try { return std::stoi(s); } catch (...) { return def; } });
        ensure_param("right_ee_channel", static_cast<int>(COM1_CHANNEL), hw_find("right_ee_channel"),
                     [](const std::string& s, int def) { try { return std::stoi(s); } catch (...) { return def; } });
        ensure_param("device_ip", std::string("192.168.1.190"), hw_find("device_ip"),
                     [](const std::string& s, const std::string& def) { (void)def; return s; });
        ensure_param("device_port", 8080, hw_find("device_port"),
                     [](const std::string& s, int def) { try { return std::stoi(s); } catch (...) { return def; } });

        // Robot operation mode: "POSITION", "JOINT_IMPEDANCE", "CART_IMPEDANCE"
        ensure_param("ctrl_mode", std::string("POSITION"), hw_find("ctrl_mode"),
                     [](const std::string& s, const std::string& def) { (void)def; return s; });
        // Drag mode: -1=don't update, 0=disable, 1=joint space drag, 2=cartesian space drag
        ensure_param("drag_mode", -1, hw_find("drag_mode"),
                     [](const std::string& s, int def) { try { return std::stoi(s); } catch (...) { return def; } });
        // Joint impedance K gains (7 values)
        ensure_double_array_sized("joint_k_gains", kDefaultJointKGains, 7);
        // Joint impedance D gains (7 values)
        ensure_double_array_sized("joint_d_gains", kDefaultJointDGains, 7);
        // Cartesian impedance K gains (7 values)
        ensure_double_array_sized("cart_k_gains", kDefaultCartKGains, 7);
        // Cartesian impedance D gains (7 values)
        ensure_double_array_sized("cart_d_gains", kDefaultCartDGains, 7);
        // Cartesian impedance type
        ensure_param("cart_type", 2, hw_find("cart_type"),
                     [](const std::string& s, int def) { try { return std::stoi(s); } catch (...) { return def; } });
        // Joint speed and acceleration limits
        ensure_param("max_joint_speed", 10.0, hw_find("max_joint_speed"),
                     [](const std::string& s, double def) { try { return std::stod(s); } catch (...) { return def; } });
        ensure_param("max_joint_acceleration", 10.0, hw_find("max_joint_acceleration"),
                     [](const std::string& s, double def) { try { return std::stod(s); } catch (...) { return def; } });
        ensure_param("use_drag_mode", false, hw_find("use_drag_mode"),
                     [](const std::string& s, bool def) { return parseBoolLoose(s, def); });
        ensure_param("use_async_tool_comm", true, hw_find("use_async_tool_comm"),
                     [](const std::string& s, bool def) { return parseBoolLoose(s, def); });
        // Per-arm brake: true=force brake release (松闸); false=engage brake (抱闸)
        ensure_param("left_brake_release", false, hw_find("left_brake_release"),
                     [](const std::string& s, bool def) { return parseBoolLoose(s, def); });
        ensure_param("right_brake_release", false, hw_find("right_brake_release"),
                     [](const std::string& s, bool def) { return parseBoolLoose(s, def); });
        // Gripper tool parameters - declared as std::vector<double> (array of doubles)
        // kineParam: kinematic parameters (6 values), dynPara: dynamic parameters (10 values)
        ensure_double_array_sized("left_kine_param", kDefaultLeftKineParam, 6);
        ensure_double_array_sized("left_dyn_param", kDefaultLeftDynParam, 10);
        ensure_double_array_sized("right_kine_param", kDefaultRightKineParam, 6);
        ensure_double_array_sized("right_dyn_param", kDefaultRightDynParam, 10);

        ensure_param("left_tool_torque", 1.0, hw_find("left_tool_torque"),
                     [](const std::string& s, double def) { try { return std::stod(s); } catch (...) { return def; } });
        ensure_param("left_tool_velocity", 1.0, hw_find("left_tool_velocity"),
                     [](const std::string& s, double def) { try { return std::stod(s); } catch (...) { return def; } });
        ensure_param("right_tool_torque", 1.0, hw_find("right_tool_torque"),
                     [](const std::string& s, double def) { try { return std::stod(s); } catch (...) { return def; } });
        ensure_param("right_tool_velocity", 1.0, hw_find("right_tool_velocity"),
                     [](const std::string& s, double def) { try { return std::stod(s); } catch (...) { return def; } });

        // KWR75 FT on COM2 (ch3); gripper on COM1 (ch2). Independent COM buses per SDK channel.
        const auto kwr75_defaults = Kwr75FtConfig::defaults();
        ensure_param("kwr75_ft_enabled", kwr75_defaults.enabled, hw_find("kwr75_ft_enabled"),
                     [](const std::string& s, bool def) { return parseBoolLoose(s, def); });
        ensure_param("left_ft_enabled", kwr75_defaults.left_enabled, hw_find("left_ft_enabled"),
                     [](const std::string& s, bool def) { return parseBoolLoose(s, def); });
        ensure_param("right_ft_enabled", kwr75_defaults.right_enabled, hw_find("right_ft_enabled"),
                     [](const std::string& s, bool def) { return parseBoolLoose(s, def); });
        ensure_param("left_ft_channel", static_cast<int>(kwr75_defaults.left_channel), hw_find("left_ft_channel"),
                     [](const std::string& s, int def) { try { return std::stoi(s); } catch (...) { return def; } });
        ensure_param("right_ft_channel", static_cast<int>(kwr75_defaults.right_channel), hw_find("right_ft_channel"),
                     [](const std::string& s, int def) { try { return std::stoi(s); } catch (...) { return def; } });
        ensure_param("ft_poll_interval_ms", kwr75_defaults.poll_interval_ms, hw_find("ft_poll_interval_ms"),
                     [](const std::string& s, int def) { try { return std::stoi(s); } catch (...) { return def; } });
        ensure_param("ft_command_code", static_cast<int>(kwr75_defaults.command_code), hw_find("ft_command_code"),
                     [](const std::string& s, int def) { try { return std::stoi(s); } catch (...) { return def; } });
        ensure_param("ft_convert_to_si", kwr75_defaults.convert_to_si, hw_find("ft_convert_to_si"),
                     [](const std::string& s, bool def) { return parseBoolLoose(s, def); });
        ensure_param("ft_gravity", kwr75_defaults.gravity, hw_find("ft_gravity"),
                     [](const std::string& s, double def) { try { return std::stod(s); } catch (...) { return def; } });
        ensure_param("ft_warmup_timeout_ms", kwr75_defaults.warmup_timeout_ms, hw_find("ft_warmup_timeout_ms"),
                     [](const std::string& s, int def) { try { return std::stoi(s); } catch (...) { return def; } });
        ensure_param("left_ft_frame_id", kwr75_defaults.left_frame_id, hw_find("left_ft_frame_id"),
                     [](const std::string& s, const std::string& def) { (void)def; return s; });
        ensure_param("right_ft_frame_id", kwr75_defaults.right_frame_id, hw_find("right_ft_frame_id"),
                     [](const std::string& s, const std::string& def) { (void)def; return s; });
        ensure_param("left_wrench_topic", kwr75_defaults.left_wrench_topic, hw_find("left_wrench_topic"),
                     [](const std::string& s, const std::string& def) { (void)def; return s; });
        ensure_param("right_wrench_topic", kwr75_defaults.right_wrench_topic, hw_find("right_wrench_topic"),
                     [](const std::string& s, const std::string& def) { (void)def; return s; });
    }
    
    hardware_interface::CallbackReturn MarvinHardware::on_init(
        const hardware_interface::HardwareComponentInterfaceParams& params)
    {
        if (SystemInterface::on_init(params) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        node_ = get_node();
        logger_ = get_node()->get_logger();
        RCLCPP_INFO(get_logger(), "Initializing Marvin Hardware Interface...");

        // Hardware error topic (heartbeat offline, etc.)
        hardware_error_pub_ = node_->create_publisher<std_msgs::msg::Int64>("/Base_HardwareError", rclcpp::SystemDefaultsQoS());
        fsm_command_pub_ = node_->create_publisher<std_msgs::msg::Int32>("/fsm_command", rclcpp::SystemDefaultsQoS());
        for (auto& f : tool_has_valid_state_)
            f.store(false);

        // 先把 hardware_parameters 的初始值灌入并声明成 ROS2 参数（便于后续动态修改）
        declare_node_parameters();

        // Virtual FT sensor inputs (wrench estimated elsewhere, published as WrenchStamped)
        // These topics are optional; when not available, the FT state will stay at zeros.
        loadKwr75FtConfig();

        if (kwr75_ft_config_.enabled)
        {
            left_wrench_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
                kwr75_ft_config_.left_wrench_topic, rclcpp::SystemDefaultsQoS());
            right_wrench_pub_ = node_->create_publisher<geometry_msgs::msg::WrenchStamped>(
                kwr75_ft_config_.right_wrench_topic, rclcpp::SystemDefaultsQoS());
            const std::string pub_mode = kwr75_ft_config_.poll_interval_ms > 0
                ? ("poll " + std::to_string(kwr75_ft_config_.poll_interval_ms) + "ms")
                : "synced to ros2_control read()";
            RCLCPP_INFO(get_logger(),
                        "KWR75 FT enabled: left/right channel=%ld/%ld, publish=%s",
                        kwr75_ft_config_.left_channel, kwr75_ft_config_.right_channel,
                        pub_mode.c_str());
        }
        else
        {
            const auto cb_left = [this](geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(wrench_mutex_);
                left_ft_state_[0] = msg->wrench.force.x;
                left_ft_state_[1] = msg->wrench.force.y;
                left_ft_state_[2] = msg->wrench.force.z;
                left_ft_state_[3] = msg->wrench.torque.x;
                left_ft_state_[4] = msg->wrench.torque.y;
                left_ft_state_[5] = msg->wrench.torque.z;
            };
            const auto cb_right = [this](geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
                std::lock_guard<std::mutex> lock(wrench_mutex_);
                right_ft_state_[0] = msg->wrench.force.x;
                right_ft_state_[1] = msg->wrench.force.y;
                right_ft_state_[2] = msg->wrench.force.z;
                right_ft_state_[3] = msg->wrench.torque.x;
                right_ft_state_[4] = msg->wrench.torque.y;
                right_ft_state_[5] = msg->wrench.torque.z;
            };

            left_wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
                kwr75_ft_config_.left_wrench_topic, rclcpp::SystemDefaultsQoS(), cb_left);
            right_wrench_sub_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
                kwr75_ft_config_.right_wrench_topic, rclcpp::SystemDefaultsQoS(), cb_right);
        }
        
        // Get the number of joints from the hardware info
        size_t num_joints = params.hardware_info.joints.size();

        // 获取机器人配置参数
        std::string arm_config_raw = get_node_param("arm_type", std::string("LEFT"));
        std::string arm_config_normalized = normalizeString(arm_config_raw);

        if (arm_config_normalized == "LEFT")
        {
            robot_arm_config_ = "LEFT";
            robot_arm_index_ = ARM_LEFT;
        }
        else if (arm_config_normalized == "RIGHT")
        {
            robot_arm_config_ = "RIGHT";
            robot_arm_index_ = ARM_RIGHT;
        }
        else if (arm_config_normalized == "DUAL")
        {
            robot_arm_config_ = "DUAL";
            robot_arm_index_ = ARM_DUAL;
        }
        else
        {
            RCLCPP_WARN(get_logger(), "Unknown arm_type '%s', using default LEFT. Valid options: LEFT, RIGHT, DUAL", arm_config_raw.c_str());
            robot_arm_config_ = "LEFT";
            robot_arm_index_ = ARM_LEFT;
        }

        // Control mode will be read from ROS2 parameter ctrl_mode ("POSITION", "JOINT_IMPEDANCE", "CART_IMPEDANCE")
        // after declare_node_parameters() is called

        // 获取末端执行器类型参数（仅使用左右独立参数）
        left_ee_type_ = normalizeString(get_node_param("left_ee_type", std::string("")));
        right_ee_type_ = normalizeString(get_node_param("right_ee_type", std::string("")));
        left_ee_channel_ = static_cast<long>(get_node_param("left_ee_channel", static_cast<int>(COM1_CHANNEL)));
        right_ee_channel_ = static_cast<long>(get_node_param("right_ee_channel", static_cast<int>(COM1_CHANNEL)));
        if (left_ee_channel_ < CAN_CHANNEL || left_ee_channel_ > 3) left_ee_channel_ = COM1_CHANNEL;
        if (right_ee_channel_ < CAN_CHANNEL || right_ee_channel_ > 3) right_ee_channel_ = COM1_CHANNEL;
        gripper_type_.clear();
        // Keep legacy global classification aligned with mixed-mode behavior:
        // if either side is a hand, prefer hand classification so hand joint
        // arrays are fully allocated and mapped.
        if (eeTypeIsHand(left_ee_type_)) {
            gripper_type_ = left_ee_type_;
        } else if (eeTypeIsHand(right_ee_type_)) {
            gripper_type_ = right_ee_type_;
        } else if (!left_ee_type_.empty()) {
            gripper_type_ = left_ee_type_;
        } else if (!right_ee_type_.empty()) {
            gripper_type_ = right_ee_type_;
        }

        use_async_tool_comm_ = get_node_param("use_async_tool_comm", true);
        init_tool_on_startup_ = get_node_param("init_tool_on_startup", true);

        RCLCPP_INFO(get_logger(), "init_tool_on_startup: %s", init_tool_on_startup_ ? "true" : "false");

        // 获取硬件连接参数
        device_ip_ = get_node_param("device_ip", std::string("192.168.1.190"));
        device_port_ = get_node_param("device_port", 8080);
        RCLCPP_INFO(get_logger(), "Device IP: %s, Port: %d", device_ip_.c_str(), device_port_);

        // 获取其他参数（将在 declare_node_parameters() 之后从节点参数读取）
   
        RCLCPP_INFO(get_logger(), "Initializing %zu joints", num_joints);

        // Resize all arrays based on the number of joints
        hw_position_commands_.resize(num_joints, 0.0);
        hw_velocity_commands_.resize(num_joints, 0.0);
        hw_position_states_.resize(num_joints, 0.0);
        hw_velocity_states_.resize(num_joints, 0.0);
        hw_effort_states_.resize(num_joints, 0.0);
        // Pre-allocate command buffer used in write() to avoid per-cycle allocations
        hw_commands_deg_buffer_.resize(num_joints, 0.0);

        // 初始化夹爪参数 如果有的话
        // 只有当 gripper_type 不为空且不为 "none" 时才考虑夹爪配置
        has_gripper_ = false;
        gripper_joint_index_ = -1;
        gripper_joint_name_ = {};
        
        // 无论是否有夹爪，都需要调用contains_tool()来填充joint_names_向量
        // 该函数会根据gripper_type_决定是否检测夹爪关节，并将非夹爪关节添加到joint_names_中
        contains_tool();
        RCLCPP_INFO(get_logger(), "has_gripper_: %d", has_gripper_);
        RCLCPP_INFO(get_logger(), "gripper_type_: %s", gripper_type_.c_str());
        
        // 构建末端执行器对象（gripper 或 hand）与其 command/state 缓存
        tool_ptr_.clear();
        tool_is_left_side_.clear();
        tool_ee_types_.clear();
        const bool has_left_ee = !left_ee_type_.empty() && left_ee_type_ != "NONE";
        const bool has_right_ee = !right_ee_type_.empty() && right_ee_type_ != "NONE";
        const bool has_any_ee = has_left_ee || has_right_ee;

        if (has_gripper_ && has_any_ee)
        {
            // Keep tool indices stable in DUAL: [0]=left(A), [1]=right(B).
            // 数组大小与 URDF 导出的关节数一致（避免单侧时 command[0] 与 tool_idx 错位）
            const size_t array_size = gripper_joint_name_.size();

            gripper_position_command_.assign(array_size, -1.0);
            gripper_effort_command_.assign(array_size, 1.0);
            gripper_velocity_command_.assign(array_size, 1.0);
            gripper_position_.assign(array_size, 0.0);
            gripper_velocity_.assign(array_size, 0.0);
            gripper_effort_.assign(array_size, 0.0);
            last_gripper_command_.assign(array_size, -1.0);
            last_gripper_effort_ack_.assign(array_size, std::numeric_limits<double>::quiet_NaN());
            last_gripper_velocity_ack_.assign(array_size, std::numeric_limits<double>::quiet_NaN());
            gripper_stopped_.assign(array_size, true);
            gripper_previous_position_.assign(array_size, std::numeric_limits<double>::quiet_NaN());
            gripper_stable_count_.assign(array_size, 0);
            tool_ptr_.resize(2);
            tool_is_left_side_.assign(2, false);
            tool_ee_types_.assign(2, "");

            // Create tool objects (hand or gripper) using unified createTool method
            if (robot_arm_index_ == ARM_LEFT)
            {
                // tool_idx=0 is always left/A.
                tool_ptr_[0] = createTool(MarvinRs485Bus::clearA(), MarvinRs485Bus::sendA(), MarvinRs485Bus::getA(), 0, left_ee_type_, left_ee_channel_);
                tool_is_left_side_[0] = true;
                tool_ee_types_[0] = left_ee_type_;
            }
            else if (robot_arm_index_ == ARM_RIGHT)
            {
                // tool_idx=1 is always right/B, even if left has no tool.
                tool_ptr_[1] = createTool(MarvinRs485Bus::clearB(), MarvinRs485Bus::sendB(), MarvinRs485Bus::getB(), 1, right_ee_type_, right_ee_channel_);
                tool_is_left_side_[1] = false;
                tool_ee_types_[1] = right_ee_type_;
            }
            else if (robot_arm_index_ == ARM_DUAL)
            {
                // Dual arm: fixed indices [0]=A(left), [1]=B(right); keep nullptr for disabled sides.
                if (has_left_ee)
                {
                    tool_ptr_[0] = createTool(MarvinRs485Bus::clearA(), MarvinRs485Bus::sendA(), MarvinRs485Bus::getA(), 0, left_ee_type_, left_ee_channel_);
                    tool_is_left_side_[0] = true;
                    tool_ee_types_[0] = left_ee_type_;
                }
                if (has_right_ee)
                {
                    tool_ptr_[1] = createTool(MarvinRs485Bus::clearB(), MarvinRs485Bus::sendB(), MarvinRs485Bus::getB(), 1, right_ee_type_, right_ee_channel_);
                    tool_is_left_side_[1] = false;
                    tool_ee_types_[1] = right_ee_type_;
                }
            }

            const size_t created_tool_count =
                static_cast<size_t>(toolAt(0) != nullptr) + static_cast<size_t>(toolAt(1) != nullptr);
            RCLCPP_INFO(get_logger(),
                        "%s init: arm_type=%s, tool_type=%s, detected_joints=%zu, created_tools=%zu",
                        toolTypeLogName(),
                        robot_arm_config_.c_str(), gripper_type_.c_str(), gripper_joint_name_.size(), created_tool_count);
        }

        // Initialize hardware connection status
        hardware_connected_ = false;
        
        // 读取关节速度和加速度限制参数（如果不存在则自动声明）
        max_joint_speed_ = get_node_param("max_joint_speed", 10.0);
        max_joint_acceleration_ = get_node_param("max_joint_acceleration", 10.0);
        
        // Read ctrl_mode parameter and set robot_ctrl_mode_ string accordingly
        std::string ctrl_mode_raw = get_node_param("ctrl_mode", std::string("POSITION"));
        robot_ctrl_mode_ = normalizeString(ctrl_mode_raw);
        RCLCPP_INFO(get_logger(), "Robot control mode set to: %s (from ctrl_mode parameter: %s)", robot_ctrl_mode_.c_str(), ctrl_mode_raw.c_str());

        // 初始化阻抗参数缓存（供 setArmCtrlInternal 使用）
        joint_k_gains_ = get_node_param("joint_k_gains", kDefaultJointKGains);
        joint_d_gains_ = get_node_param("joint_d_gains", kDefaultJointDGains);
        cart_k_gains_  = get_node_param("cart_k_gains", kDefaultCartKGains);
        cart_d_gains_  = get_node_param("cart_d_gains", kDefaultCartDGains);

        RCLCPP_INFO(get_logger(), "Robot arm configuration: %s", robot_arm_config_.c_str());
        RCLCPP_INFO(get_logger(), "EE type: left=%s, right=%s",
                    left_ee_type_.empty() ? "none" : left_ee_type_.c_str(),
                    right_ee_type_.empty() ? "none" : right_ee_type_.c_str());
        
        // Initialize kineParam and dynPara - read from node parameters (如果不存在则自动声明)
        leftkineParam_ = get_node_param("left_kine_param", kDefaultLeftKineParam);
        if (leftkineParam_.size() != 6) leftkineParam_.resize(6, 0.0);
        
        leftdynParam_ = get_node_param("left_dyn_param", kDefaultLeftDynParam);
        if (leftdynParam_.size() != 10)
        {
            if (has_gripper_ && !gripper_type_.empty() && gripper_type_ != "NONE")
            {
                leftdynParam_ = kDefaultLeftDynParam;
            }
            else
            {
                leftdynParam_.resize(10, 0.0);
            }
        }
        
        rightkineParam_ = get_node_param("right_kine_param", kDefaultRightKineParam);
        if (rightkineParam_.size() != 6) rightkineParam_.resize(6, 0.0);
        
        rightdynParam_ = get_node_param("right_dyn_param", kDefaultRightDynParam);
        if (rightdynParam_.size() != 10)
        {
            if (has_gripper_ && !gripper_type_.empty() && gripper_type_ != "NONE")
            {
                rightdynParam_ = kDefaultRightDynParam;
            }
            else
            {
                rightdynParam_.resize(10, 0.0);
            }
        }
        
        param_callback_handle_ = node_->add_on_set_parameters_callback(
                std::bind(&MarvinHardware::paramCallback, this, std::placeholders::_1));
        syncToolDynamicsFromNodeParams();
        RCLCPP_INFO(get_logger(), "Robot configuration parameters ready. Use 'ros2 param set' to change configuration dynamically.");
            
        RCLCPP_INFO(get_logger(), "Marvin Hardware Interface initialized successfully");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    size_t MarvinHardware::mappedToolIndexForJoint(size_t k) const
    {
        if (k >= gripper_joint_name_.size())
        {
            return 0;
        }
        std::string name_lower = gripper_joint_name_[k];
        std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);
        if (tool_type_ == ToolType::Hand)
        {
            return (toolCount() >= 2 && name_lower.find("right_hand_") != std::string::npos) ? 1u : 0u;
        }
        return (name_lower.find("right") != std::string::npos) ? 1u : 0u;
    }

    bool MarvinHardware::gripperJointBelongsToTool(size_t k, size_t tool_idx) const
    {
        return mappedToolIndexForJoint(k) == tool_idx;
    }

    static bool isToolDynamicsParamName(const std::string& name)
    {
        return name == "left_tool_torque" || name == "left_tool_velocity"
            || name == "right_tool_torque" || name == "right_tool_velocity";
    }

    void MarvinHardware::applyAllToolDynamics(const std::unordered_map<std::string, double>* pending)
    {
        auto read_tool_param = [this, pending](const std::string& name, double default_val) {
            if (pending)
            {
                const auto it = pending->find(name);
                if (it != pending->end())
                {
                    return it->second;
                }
            }
            return get_node_param(name, default_val);
        };

        const double left_torque = std::clamp(read_tool_param("left_tool_torque", 1.0), 0.0, 1.0);
        const double left_velocity = std::clamp(read_tool_param("left_tool_velocity", 1.0), 0.0, 1.0);
        const double right_torque = std::clamp(read_tool_param("right_tool_torque", 1.0), 0.0, 1.0);
        const double right_velocity = std::clamp(read_tool_param("right_tool_velocity", 1.0), 0.0, 1.0);

        const size_t n = std::max({gripper_joint_name_.size(), gripper_effort_command_.size(),
                                   gripper_velocity_command_.size()});
        for (size_t k = 0; k < n; ++k)
        {
            const size_t side = mappedToolIndexForJoint(k);
            const double torque = (side == 0) ? left_torque : right_torque;
            const double velocity = (side == 0) ? left_velocity : right_velocity;
            if (k < gripper_effort_command_.size())
            {
                gripper_effort_command_[k] = torque;
            }
            if (k < gripper_velocity_command_.size())
            {
                gripper_velocity_command_[k] = velocity;
            }
            if (k < last_gripper_effort_ack_.size())
            {
                last_gripper_effort_ack_[k] = std::numeric_limits<double>::quiet_NaN();
            }
            if (k < last_gripper_velocity_ack_.size())
            {
                last_gripper_velocity_ack_[k] = std::numeric_limits<double>::quiet_NaN();
            }
        }
    }

    void MarvinHardware::syncToolDynamicsFromNodeParams()
    {
        if (!node_ || !has_gripper_)
        {
            return;
        }
        applyAllToolDynamics();
    }

rcl_interfaces::msg::SetParametersResult
MarvinHardware::paramCallback(const std::vector<rclcpp::Parameter> & params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    // Track if we need to apply configuration
    bool need_config_update = false;
    bool ctrl_mode_changed = false;
    std::string ctrl_mode = "";
    int drag_mode = -1;
    int cart_type = -1;
    double max_joint_speed = -1.0;
    double max_joint_acceleration = -1.0;
    std::vector<double> joint_k_gains;
    std::vector<double> joint_d_gains;
    std::vector<double> cart_k_gains;
    std::vector<double> cart_d_gains;

    std::unordered_map<std::string, double> tool_dyn_batch;
    for (const auto& param : params)
    {
        const std::string& pname = param.get_name();
        if (!isToolDynamicsParamName(pname))
        {
            continue;
        }
        const double v = param.as_double();
        if (v < 0.0 || v > 1.0)
        {
            result.successful = false;
            result.reason = pname + " must be in [0.0, 1.0]";
            return result;
        }
        tool_dyn_batch[pname] = v;
    }
    if (!tool_dyn_batch.empty())
    {
        applyAllToolDynamics(&tool_dyn_batch);
        for (const auto& entry : tool_dyn_batch)
        {
            RCLCPP_INFO(get_logger(), "Updated %s = %.3f", entry.first.c_str(), entry.second);
        }
    }

    for (const auto & param : params) {
        const std::string& pname = param.get_name();
        if (isToolDynamicsParamName(pname))
        {
            continue;
        }

        if (param.get_name() == "left_brake_release" || param.get_name() == "right_brake_release") {
            if (!hardware_connected_) {
                result.successful = false;
                result.reason = "Hardware not connected";
                return result;
            }
            const bool is_left = (param.get_name() == "left_brake_release");
            const bool arm_configured = is_left
                ? (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL)
                : (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL);
            if (!arm_configured) {
                RCLCPP_WARN(get_logger(), "%s ignored: arm not configured (arm_type=%s)",
                            param.get_name().c_str(), robot_arm_config_.c_str());
                continue;
            }

            if (robot_ctrl_mode_ != "POWER_OFF") {
                result.successful = false;
                result.reason = "ctrl_mode must be POWER_OFF before brake operation (current: "
                                  + robot_ctrl_mode_ + ")";
                RCLCPP_WARN(get_logger(), "Rejected %s: %s",
                            param.get_name().c_str(), result.reason.c_str());
                return result;
            }

            const bool release = param.as_bool();
            const long sdk_value = release ? 2 : 1;  // 2=松闸, 1=抱闸
            const int arm_index = is_left ? ARM_LEFT : ARM_RIGHT;
            char name[30] = "";
            std::snprintf(name, sizeof(name), "BRAK%d", arm_index);

            OnClearSet();
            if (release) {
                if (is_left) OnSetTargetState_A(0);
                else OnSetTargetState_B(0);
            }
            OnSetIntPara(name, sdk_value);
            OnSetSend();
            usleep(100000);

            if (is_left) left_brake_released_ = release;
            else right_brake_released_ = release;

            RCLCPP_INFO(get_logger(), "%s arm brake: %s",
                        is_left ? "Left" : "Right",
                        release ? "BRAKE_RELEASE" : "BRAKE");
            continue;
        }

        // Other parameters require hardware connection
        if (!hardware_connected_) {
            result.successful = false;
            result.reason = "Hardware not connected";
            return result;
        }
        if (param.get_name() == "ctrl_mode") {
            std::string ctrl_mode_str = param.as_string();
            const int mode = ctrlModeStringToMode(ctrl_mode_str, get_logger());
            if (mode != 4) {
                const bool left_released = (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL)
                    && left_brake_released_;
                const bool right_released = (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL)
                    && right_brake_released_;
                if (left_released || right_released) {
                    std::string arms;
                    if (left_released) arms += "left";
                    if (right_released) {
                        if (!arms.empty()) arms += "+";
                        arms += "right";
                    }
                    result.successful = false;
                    result.reason = "brake released on " + arms + " arm(s), only POWER_OFF is allowed";
                    RCLCPP_WARN(get_logger(), "Rejected ctrl_mode=%s: %s",
                                ctrl_mode_str.c_str(), result.reason.c_str());
                    return result;
                }
            }
            ctrl_mode = modeToCtrlModeString(mode);
            robot_ctrl_mode_ = ctrl_mode;
            if (mode != 4) last_active_ctrl_mode_ = ctrl_mode;
            need_config_update = true;
            ctrl_mode_changed = true;
            RCLCPP_INFO(get_logger(), "ctrl_mode parameter changed to: %s", ctrl_mode.c_str());
            if (mode != 4) {
                if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL) left_brake_released_ = false;
                if (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL) right_brake_released_ = false;
            }
        }
        else if (param.get_name() == "drag_mode") {
            drag_mode = param.as_int();
            need_config_update = true;
            RCLCPP_INFO(get_logger(), "Drag mode parameter changed to: %d", drag_mode);
        }
        else if (param.get_name() == "joint_k_gains") {
            joint_k_gains = param.as_double_array();
            if (joint_k_gains.size() != 7) {
                result.successful = false;
                result.reason = "joint_k_gains must have exactly 7 values";
                return result;
            }
            need_config_update = true;
        }
        else if (param.get_name() == "joint_d_gains") {
            joint_d_gains = param.as_double_array();
            if (joint_d_gains.size() != 7) {
                result.successful = false;
                result.reason = "joint_d_gains must have exactly 7 values";
                return result;
            }
            need_config_update = true;
        }
        else if (param.get_name() == "cart_k_gains") {
            cart_k_gains = param.as_double_array();
            if (cart_k_gains.size() != 7) {
                result.successful = false;
                result.reason = "cart_k_gains must have exactly 7 values";
                return result;
            }
            need_config_update = true;
        }
        else if (param.get_name() == "cart_d_gains") {
            cart_d_gains = param.as_double_array();
            if (cart_d_gains.size() != 7) {
                result.successful = false;
                result.reason = "cart_d_gains must have exactly 7 values";
                return result;
            }
            need_config_update = true;
        }
        else if (param.get_name() == "cart_type") {
            cart_type = param.as_int();
            need_config_update = true;
        }
        else if (param.get_name() == "max_joint_speed") {
            max_joint_speed = param.as_double();
            if (max_joint_speed <= 0.0) {
                result.successful = false;
                result.reason = "max_joint_speed must be > 0";
                return result;
            }
            need_config_update = true;
        }
        else if (param.get_name() == "max_joint_acceleration") {
            max_joint_acceleration = param.as_double();
            if (max_joint_acceleration <= 0.0) {
                result.successful = false;
                result.reason = "max_joint_acceleration must be > 0";
                return result;
            }
            need_config_update = true;
        }
        else if (param.get_name() == "left_dyn_param") {
            std::vector<double> new_left_dyn = param.as_double_array();
            if (new_left_dyn.size() != 10) {
                result.successful = false;
                result.reason = "left_dyn_param must have exactly 10 values";
                return result;
            }
            leftdynParam_ = new_left_dyn;
            RCLCPP_INFO(get_logger(), "Left dynParam updated via parameter");
            // Update the tool parameters on the hardware
            if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL) {
                setArmCtrlInternal(ARM_LEFT);
            }
        }
        else if (param.get_name() == "right_dyn_param") {
            std::vector<double> new_right_dyn = param.as_double_array();
            if (new_right_dyn.size() != 10) {
                result.successful = false;
                result.reason = "right_dyn_param must have exactly 10 values";
                return result;
            }
            rightdynParam_ = new_right_dyn;
            RCLCPP_INFO(get_logger(), "Right dynParam updated via parameter");
            // Update the tool parameters on the hardware
            if (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL) {
                setArmCtrlInternal(ARM_RIGHT);
            }
        }
        else if (param.get_name() == "left_kine_param") {
            std::vector<double> new_left_kine = param.as_double_array();
            if (new_left_kine.size() != 6) {
                result.successful = false;
                result.reason = "left_kine_param must have exactly 6 values";
                return result;
            }
            leftkineParam_ = new_left_kine;
            RCLCPP_INFO(get_logger(), "Left kineParam updated via parameter");
        }
        else if (param.get_name() == "right_kine_param") {
            std::vector<double> new_right_kine = param.as_double_array();
            if (new_right_kine.size() != 6) {
                result.successful = false;
                result.reason = "right_kine_param must have exactly 6 values";
                return result;
            }
            rightkineParam_ = new_right_kine;
            RCLCPP_INFO(get_logger(), "Right kineParam updated via parameter");
        }
    }

    if (need_config_update) {
        const int mode = ctrl_mode.empty() ? -1 : ctrlModeStringToMode(ctrl_mode, get_logger());
        applyRobotConfiguration(mode, drag_mode, cart_type,
                              max_joint_speed, max_joint_acceleration,
                              joint_k_gains, joint_d_gains, cart_k_gains, cart_d_gains);

        if (ctrl_mode_changed) {
            if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL) {
                setArmCtrlInternal(ARM_LEFT);
            }
            if (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL) {
                setArmCtrlInternal(ARM_RIGHT);
            }
        }
    }

    return result;
}

void MarvinHardware::applyRobotConfiguration(int mode, int drag_mode, int cart_type,
                                              double max_joint_speed, double max_joint_acceleration,
                                              const std::vector<double>& joint_k_gains,
                                              const std::vector<double>& joint_d_gains,
                                              const std::vector<double>& cart_k_gains,
                                              const std::vector<double>& cart_d_gains)
{
    const auto send_and_sleep = []() {
        OnSetSend();
        usleep(100000);
    };
    const auto apply_drag_mode_if_needed = [&](bool update_left, bool update_right) {
        if (drag_mode >= 0) {
            if (update_left) {
                OnSetDragSpace_A(drag_mode);
            }
            if (update_right) {
                OnSetDragSpace_B(drag_mode);
            }
        }
    };
    const auto apply_joint_limits = [&](bool update_left, bool update_right) {
        OnClearSet();
        if (update_left) {
            OnSetJointLmt_A(static_cast<int>(max_joint_speed), static_cast<int>(max_joint_acceleration));
        }
        if (update_right) {
            OnSetJointLmt_B(static_cast<int>(max_joint_speed), static_cast<int>(max_joint_acceleration));
        }
        send_and_sleep();
    };

    // Get current parameter values if not provided
    if (mode == -1) {
        const std::string ctrl_mode_str = get_node_param("ctrl_mode", std::string("POSITION"));
        mode = ctrlModeStringToMode(ctrl_mode_str, get_logger());
    }
    robot_ctrl_mode_ = modeToCtrlModeString(mode);
    if (drag_mode == -1) {
        drag_mode = get_node_param("drag_mode", -1);
    }
    if (cart_type == -1) {
        cart_type = get_node_param("cart_type", 2);
    }
    if (max_joint_speed <= 0.0) {
        max_joint_speed = get_node_param("max_joint_speed", 10.0);
    }
    if (max_joint_acceleration <= 0.0) {
        max_joint_acceleration = get_node_param("max_joint_acceleration", 10.0);
    }

    // Use provided KD or get from parameters or defaults
    std::vector<double> final_joint_k = (!joint_k_gains.empty()) ? joint_k_gains :
        (get_node_param("joint_k_gains", kDefaultJointKGains).size() == 7) ?
        get_node_param("joint_k_gains", kDefaultJointKGains) : kDefaultJointKGains;
    std::vector<double> final_joint_d = (!joint_d_gains.empty()) ? joint_d_gains :
        (get_node_param("joint_d_gains", kDefaultJointDGains).size() == 7) ?
        get_node_param("joint_d_gains", kDefaultJointDGains) : kDefaultJointDGains;
    std::vector<double> final_cart_k = (!cart_k_gains.empty()) ? cart_k_gains :
        (get_node_param("cart_k_gains", kDefaultCartKGains).size() == 7) ?
        get_node_param("cart_k_gains", kDefaultCartKGains) : kDefaultCartKGains;
    std::vector<double> final_cart_d = (!cart_d_gains.empty()) ? cart_d_gains :
        (get_node_param("cart_d_gains", kDefaultCartDGains).size() == 7) ?
        get_node_param("cart_d_gains", kDefaultCartDGains) : kDefaultCartDGains;

    // 同步缓存：供 setArmCtrlInternal 使用（避免只从 hardware_parameters 读一次导致无法动态更新）
    max_joint_speed_ = max_joint_speed;
    max_joint_acceleration_ = max_joint_acceleration;
    joint_k_gains_ = final_joint_k;
    joint_d_gains_ = final_joint_d;
    cart_k_gains_ = final_cart_k;
    cart_d_gains_ = final_cart_d;

    // Determine which arms to update (统一使用 arm_type / robot_arm_index_)
    bool update_left = (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL);
    bool update_right = (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL);

    // Process based on selected mode
    if (mode == 1) {
        // Position mode
        OnClearSet();
        if (update_left) {
            OnSetTargetState_A(1);  // Position mode
        }
        if (update_right) {
            OnSetTargetState_B(1);  // Position mode
        }
        send_and_sleep();
        apply_joint_limits(update_left, update_right);
        
        RCLCPP_INFO(get_logger(), "Set to position mode with speed=%.1f, acceleration=%.1f", 
                    max_joint_speed, max_joint_acceleration);
        
    } else if (mode == 2) {
        // Joint impedance mode
        double K[7], D[7];
        for (int i = 0; i < 7; i++) {
            K[i] = final_joint_k[i];
            D[i] = final_joint_d[i];
        }
        
        OnClearSet();
        if (update_left) {
            OnSetTargetState_A(3);  // Torque mode
            OnSetImpType_A(1);      // Joint impedance
            OnSetJointKD_A(K, D);
        }
        if (update_right) {
            OnSetTargetState_B(3);  // Torque mode
            OnSetImpType_B(1);      // Joint impedance
            OnSetJointKD_B(K, D);
        }
        apply_drag_mode_if_needed(update_left, update_right);
        send_and_sleep();
        apply_joint_limits(update_left, update_right);
        
        RCLCPP_INFO(get_logger(), "Set to joint impedance mode with KD=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f], speed=%.1f, acceleration=%.1f",
                    K[0], K[1], K[2], K[3], K[4], K[5], K[6], max_joint_speed, max_joint_acceleration);
        
    } else if (mode == 3) {
        // Cartesian impedance mode
        double K[7], D[7];
        for (int i = 0; i < 7; i++) {
            K[i] = final_cart_k[i];
            D[i] = final_cart_d[i];
        }
        
        OnClearSet();
        if (update_left) {
            OnSetTargetState_A(3);  // Torque mode
            OnSetImpType_A(2);      // Cartesian impedance
            OnSetCartKD_A(K, D, cart_type);
        }
        if (update_right) {
            OnSetTargetState_B(3);  // Torque mode
            OnSetImpType_B(2);      // Cartesian impedance
            // Right arm uses 6 values for cartesian
            double K_right[6], D_right[6];
            for (int i = 0; i < 6; i++) {
                K_right[i] = final_cart_k[i];
                D_right[i] = final_cart_d[i];
            }
            OnSetCartKD_B(K_right, D_right, cart_type);
        }
        apply_drag_mode_if_needed(update_left, update_right);
        send_and_sleep();
        apply_joint_limits(update_left, update_right);
        
        RCLCPP_INFO(get_logger(), "Set to cartesian impedance mode with KD=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f], speed=%.1f, acceleration=%.1f",
                    K[0], K[1], K[2], K[3], K[4], K[5], K[6], max_joint_speed, max_joint_acceleration);
    } else if (mode == 4) {
        // POWER_OFF: 下使能；A/B 分两次下发，避免同包卡顿
        if (update_left) {
            OnClearSet();
            OnSetTargetState_A(0);
            OnSetSend();
            usleep(100000);
        }
        if (update_right) {
            OnClearSet();
            OnSetTargetState_B(0);
            OnSetSend();
            usleep(100000);
        }
        RCLCPP_INFO(get_logger(), "Set to POWER_OFF mode (左右臂下使能)");
    }
}


    void MarvinHardware::setArmCtrlInternal(int arm_index)
    {
        const bool is_left = (arm_index == ARM_LEFT);
        if (!is_left && arm_index != ARM_RIGHT)
        {
            RCLCPP_WARN(get_logger(), "setArmCtrlInternal called with invalid arm_index=%d", arm_index);
            return;
        }

        auto& kine = is_left ? leftkineParam_ : rightkineParam_;
        auto& dyn = is_left ? leftdynParam_ : rightdynParam_;

        // Ensure parameters are initialized
        if (kine.size() != 6) kine.resize(6, 0.0);
        if (dyn.size() != 10) dyn.resize(10, 0.0);

        const auto clear_err = is_left ? OnClearErr_A : OnClearErr_B;
        const auto set_target_state = is_left ? OnSetTargetState_A : OnSetTargetState_B;
        const auto set_tool = is_left ? OnSetTool_A : OnSetTool_B;
        const auto set_joint_kd = is_left ? OnSetJointKD_A : OnSetJointKD_B;
        const auto set_cart_kd = is_left ? OnSetCartKD_A : OnSetCartKD_B;
        const auto set_imp_type = is_left ? OnSetImpType_A : OnSetImpType_B;
        const auto set_joint_lmt = is_left ? OnSetJointLmt_A : OnSetJointLmt_B;

        OnClearSet();
        clear_err();
        OnSetSend();
        usleep(100000);

        if (robot_ctrl_mode_ == "POSITION")
        {
            OnClearSet();
            set_tool(kine.data(), dyn.data());
            set_target_state(1); // 1:position mode
            OnSetSend();
            usleep(100000);
        }
        else if (robot_ctrl_mode_ == "POWER_OFF")
        {
            OnClearSet();
            set_target_state(0); // 0:下使能 / servo off
            OnSetSend();
            usleep(100000);
        }
        else
        {
            OnClearSet();
            set_tool(kine.data(), dyn.data());
            set_target_state(3); // 3:torque mode

            if (robot_ctrl_mode_ == "JOINT_IMPEDANCE")
            {
                const bool valid = (joint_k_gains_.size() >= 7 && joint_d_gains_.size() >= 7);
                if (!valid)
                {
                    RCLCPP_WARN(get_logger(), "Joint impedance parameters size mismatch, using defaults");
                }
                const auto& K_src = valid ? joint_k_gains_ : kDefaultJointKGains;
                const auto& D_src = valid ? joint_d_gains_ : kDefaultJointDGains;
                double K[7];
                double D[7];
                for (size_t i = 0; i < 7; i++)
                {
                    K[i] = K_src[i];
                    D[i] = D_src[i];
                }
                set_joint_kd(K, D);
                set_imp_type(1);
            }
            else if (robot_ctrl_mode_ == "CART_IMPEDANCE")
            {
                const bool valid = (cart_k_gains_.size() >= 7 && cart_d_gains_.size() >= 7);
                if (!valid)
                {
                    RCLCPP_WARN(get_logger(), "Cartesian impedance parameters size mismatch, using defaults");
                }
                const auto& K_src = valid ? cart_k_gains_ : kDefaultCartKGains;
                const auto& D_src = valid ? cart_d_gains_ : kDefaultCartDGains;
                double K[7];
                double D[7];
                for (size_t i = 0; i < 7; i++)
                {
                    K[i] = K_src[i];
                    D[i] = D_src[i];
                }
                set_cart_kd(K, D, 2);
                set_imp_type(2);
            }

            OnSetSend();
            usleep(100000);
        }

        // set maximum speed and acceleration
        OnClearSet();
        set_joint_lmt(static_cast<int>(max_joint_speed_), static_cast<int>(max_joint_acceleration_));
        OnSetSend();
        usleep(100000);
    }

    std::unique_ptr<gripper_hardware_common::GripperBase> MarvinHardware::createTool(
        marvin_ros2_control::Clear485Func clear_485, 
        marvin_ros2_control::Send485Func send_485,
        marvin_ros2_control::GetChDataFunc get_ch_data,
        size_t tool_index,
        const std::string& ee_type,
        long channel)
    {
        /// 启动的时候调用
        // clear_485();
        // ee_type has been normalized to UPPERCASE by normalizeString()
        const std::string normalized_ee_type = ee_type.empty() ? gripper_type_ : ee_type;
        
        // Determine if this is left or right hand based on robot_arm_index_ and tool_index
        // For ARM_LEFT: tool_index=0 -> left hand
        // For ARM_RIGHT: tool_index=1 -> right hand  
        // For ARM_DUAL: tool_index=0 -> left hand, tool_index=1 -> right hand
        bool is_left_hand = (robot_arm_index_ == ARM_LEFT) || 
                           (robot_arm_index_ == ARM_DUAL && tool_index == 0);
        
        // Determine if end effector is a hand or gripper based on type
        // Hand types: LINKERHAND_O7, LINKERHAND_O6, LINKERHAND_L6 (all are LinkerHand/DexterousHand)
        // Also support short forms: O7, O6, L6
        static const std::set<std::string> hand_types = {
            "LINKERHAND_O7", "LINKERHAND_O6", "LINKERHAND_L6",
            "O7", "O6", "L6",
            "FREEDOM_V1", "FREEDOM_V2", "FREEDOM",
            "INSPIRE_E2", "INSPIRE", "RH56E2",
            "INSPIRE_F2", "RH56F2",
            "ERG32"
        };
        
        // Check if type contains hand indicators
        bool is_hand_type = false;
        std::string hand_model = "O7";  // Default
        
        if (hand_types.find(normalized_ee_type) != hand_types.end())
        {
            is_hand_type = true;
        }
        else if (normalized_ee_type.find("LINKERHAND") != std::string::npos ||
                 normalized_ee_type.find("HAND") != std::string::npos)
        {
            // Check for hand type patterns in the string
            is_hand_type = true;
        }
        
        if (is_hand_type)
        {
            if (normalized_ee_type == "ERG32" || normalized_ee_type.find("ERG32") != std::string::npos)
            {
                RCLCPP_INFO(get_logger(), "Creating ERG32 hand (2-DOF rotate+grip, slave: 0x%02X)",
                            gripper_hardware_common::ModbusConfig::ERG32::SLAVE_ID);
                return std::make_unique<marvin_ros2_control::ERG32Hand>(clear_485, send_485, get_ch_data);
            }
            // Extract hand model from type string and create corresponding hand object
            if (normalized_ee_type == "FREEDOM_V1" || normalized_ee_type == "FREEDOM")
            {
                hand_model = "FREEDOM_V1";
                RCLCPP_INFO(get_logger(), "Creating freedom_v1 hand (6-DOF, %s hand, slave: 0x%02X, channel: %ld)",
                           is_left_hand ? "left" : "right",
                           is_left_hand ? 0x00 : 0x01,
                           channel);
                return std::make_unique<marvin_ros2_control::FreedomHandV1>(
                    clear_485, send_485, get_ch_data, is_left_hand, channel);
            }
            else if (normalized_ee_type == "FREEDOM_V2")
            {
                hand_model = "FREEDOM_V2";
                RCLCPP_INFO(get_logger(), "Creating freedom_v2 hand (7-DOF, 9-slot protocol, %s hand, slave: 0x%02X, channel: %ld)",
                           is_left_hand ? "left" : "right",
                           is_left_hand ? 0x00 : 0x01,
                           channel);
                return std::make_unique<marvin_ros2_control::FreedomHandV2>(
                    clear_485, send_485, get_ch_data, is_left_hand, channel);
            }
            else if (normalized_ee_type == "INSPIRE_E2" ||
                     normalized_ee_type == "INSPIRE" ||
                     normalized_ee_type == "RH56E2")
            {
                hand_model = "INSPIRE_E2";
                if (channel == CAN_CHANNEL)
                {
                    RCLCPP_INFO(get_logger(), "Creating inspire_e2 CANFD hand (6-DOF, %s hand, hand_id: 0x%02X, channel: %ld)",
                               is_left_hand ? "left" : "right",
                               is_left_hand ? 0x02 : 0x01,
                               channel);
                    return std::make_unique<marvin_ros2_control::InspireHandE2Canfd>(
                        clear_485, send_485, get_ch_data, is_left_hand, channel);
                }
                RCLCPP_INFO(get_logger(), "Creating inspire_e2 hand (6-DOF, %s hand, slave: 0x%02X)",
                           is_left_hand ? "left" : "right",
                           is_left_hand ? 0x02 : 0x01);
                return std::make_unique<marvin_ros2_control::InspireHandE2>(clear_485, send_485, get_ch_data, is_left_hand);
            }
            else if (normalized_ee_type == "INSPIRE_F2" ||
                     normalized_ee_type == "RH56F2")
            {
                hand_model = "INSPIRE_F2";
                if (channel == CAN_CHANNEL)
                {
                    RCLCPP_INFO(get_logger(), "Creating inspire_f2 CANFD hand (6-DOF, %s hand, hand_id: 0x%02X, channel: %ld)",
                               is_left_hand ? "left" : "right",
                               is_left_hand ? 0x02 : 0x01,
                               channel);
                    return std::make_unique<marvin_ros2_control::InspireHandE2Canfd>(
                        clear_485, send_485, get_ch_data, is_left_hand, channel);
                }
                RCLCPP_INFO(get_logger(), "Creating inspire_f2 hand (6-DOF, %s hand, slave: 0x%02X)",
                           is_left_hand ? "left" : "right",
                           is_left_hand ? 0x02 : 0x01);
                return std::make_unique<marvin_ros2_control::InspireHandE2>(clear_485, send_485, get_ch_data, is_left_hand);
            }
            else if (normalized_ee_type.find("O7") != std::string::npos || normalized_ee_type == "O7")
            {
                hand_model = "O7";
                RCLCPP_INFO(get_logger(), "Creating LinkerHand O7 (7-DOF, %s hand, slave: 0x%02X)", 
                           is_left_hand ? "left" : "right",
                           is_left_hand ? 0x28 : 0x27);
                return std::make_unique<marvin_ros2_control::DexterousHandO7>(clear_485, send_485, get_ch_data, is_left_hand);
            }
            else if (normalized_ee_type.find("L6") != std::string::npos || normalized_ee_type == "L6")
            {
                hand_model = "L6";
                RCLCPP_INFO(get_logger(), "Creating LinkerHand L6 (6-DOF, %s hand, slave: 0x%02X)", 
                           is_left_hand ? "left" : "right",
                           is_left_hand ? 0x28 : 0x27);
                return std::make_unique<marvin_ros2_control::DexterousHandL6>(clear_485, send_485, get_ch_data, is_left_hand);
            }
            else if (normalized_ee_type.find("O6") != std::string::npos || normalized_ee_type == "O6")
            {
                hand_model = "O6";
                RCLCPP_INFO(get_logger(), "Creating LinkerHand O6 (6-DOF, %s hand, slave: 0x%02X)", 
                           is_left_hand ? "left" : "right",
                           is_left_hand ? 0x28 : 0x27);
                return std::make_unique<marvin_ros2_control::DexterousHandO6>(clear_485, send_485, get_ch_data, is_left_hand);
            }
            else
            {
                // Default to O7 if model cannot be determined
                hand_model = "O7";
                RCLCPP_WARN(get_logger(), "Unknown hand model in type '%s', defaulting to O7", normalized_ee_type.c_str());
                return std::make_unique<marvin_ros2_control::DexterousHandO7>(clear_485, send_485, get_ch_data, is_left_hand);
            }
        }
        else
        {
            // Create gripper
            enum class GripperKind { JD, Changingtek90C, Changingtek90D, Changingtek120S, EincinX };

            static const std::unordered_map<std::string, GripperKind> kGripperTypeMap = {
                // JD / RG75
                {"RG75", GripperKind::JD},
                {"JDGRIPPER", GripperKind::JD},

                // EincinX electric gripper (grip axis Modbus slave 2)
                {"EINCINX", GripperKind::EincinX},
                {"EPGI180", GripperKind::EincinX},

                // Changingtek 90C variants (AG2F90_C is the common实际入参)
                {"CHANGINGTEK90C", GripperKind::Changingtek90C},
                {"AG2F90", GripperKind::Changingtek90C},
                {"AG2F90C", GripperKind::Changingtek90C},
                {"AG2F90_C", GripperKind::Changingtek90C},

                // Changingtek 90D variants
                {"CHANGINGTEK90D", GripperKind::Changingtek90D},
                {"AG2F90D", GripperKind::Changingtek90D},
                {"AG2F90_D", GripperKind::Changingtek90D},

                // Changingtek 120S (AG2F120S / AG2F120S_D)
                {"AG2F120S", GripperKind::Changingtek120S},
                {"AG2F120S_D", GripperKind::Changingtek120S},
                                            };

            const auto it = kGripperTypeMap.find(normalized_ee_type);
            const GripperKind kind = (it == kGripperTypeMap.end()) ? GripperKind::JD : it->second;

            switch (kind)
            {
                case GripperKind::JD:
                    if (it == kGripperTypeMap.end())
                    {
                        RCLCPP_WARN(get_logger(),
                                    "Unknown gripper type '%s' (normalized). Using default JDGripper.",
                                    normalized_ee_type.c_str());
                    }
                    RCLCPP_INFO(get_logger(), "Creating JD Gripper");
                    return std::make_unique<marvin_ros2_control::JDGripper>(clear_485, send_485, get_ch_data);

                case GripperKind::Changingtek90C:
                    RCLCPP_INFO(get_logger(), "Creating CHANGINGTEK90C Gripper");
                    return std::make_unique<marvin_ros2_control::ChangingtekGripper90C>(clear_485, send_485, get_ch_data);

                case GripperKind::Changingtek90D:
                    RCLCPP_INFO(get_logger(), "Creating CHANGINGTEK90D Gripper");
                    return std::make_unique<marvin_ros2_control::ChangingtekGripper90D>(clear_485, send_485, get_ch_data);

                case GripperKind::Changingtek120S:
                    if (normalized_ee_type == "AG2F120S_D")
                    {
                        RCLCPP_INFO(get_logger(), "Creating CHANGINGTEK120S_D Gripper");
                        return std::make_unique<marvin_ros2_control::ChangingtekGripper<
                            gripper_hardware_common::ModbusConfig::Changingtek120S_D>>(
                            clear_485, send_485, get_ch_data);
                    }
                    RCLCPP_INFO(get_logger(), "Creating CHANGINGTEK120S Gripper");
                    return std::make_unique<marvin_ros2_control::ChangingtekGripper120S>(clear_485, send_485, get_ch_data);

                case GripperKind::EincinX:
                    RCLCPP_INFO(get_logger(), "Creating EincinX Gripper");
                    return std::make_unique<marvin_ros2_control::EincinXGripper>(clear_485, send_485, get_ch_data);
            }

            // Defensive fallback
            RCLCPP_INFO(get_logger(), "Creating JD Gripper");
            return std::make_unique<marvin_ros2_control::JDGripper>(clear_485, send_485, get_ch_data);
        }
    }

    hardware_interface::CallbackReturn MarvinHardware::on_configure(
            const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Configuring Marvin Hardware Interface...");


        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MarvinHardware::on_cleanup(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up Marvin Hardware Interface...");

        if (hardware_connected_)
        {
            disconnectFromHardware();
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }


    const char* MarvinHardware::toolTypeLogName() const
    {
        switch (tool_type_)
        {
            case ToolType::Hand:    return "Hand";
            case ToolType::Gripper: return "Gripper";
            case ToolType::Others:  return "Others";
            case ToolType::None:    return "None";
        }
        return "None";
    }

    bool MarvinHardware::eeTypeIsHand(const std::string& ee_type) const
    {
        return ee_type == "ERG32" || ee_type.find("ERG32") != std::string::npos ||
               ee_type == "LINKERHAND_O7" || ee_type == "LINKERHAND_O6" || ee_type == "LINKERHAND_L6" ||
               ee_type == "O7" || ee_type == "O6" || ee_type == "L6" ||
               ee_type == "FREEDOM_V1" || ee_type == "FREEDOM_V2" || ee_type == "FREEDOM" ||
               ee_type == "INSPIRE_E2" || ee_type == "INSPIRE" ||
               ee_type == "RH56E2" ||
               ee_type == "INSPIRE_F2" || ee_type == "RH56F2" ||
               ee_type.find("LINKERHAND") != std::string::npos;
    }

    std::string MarvinHardware::eeTypeForTool(size_t tool_idx) const
    {
        if (tool_idx < tool_ee_types_.size())
        {
            return tool_ee_types_[tool_idx];
        }
        if (robot_arm_index_ == ARM_LEFT) return left_ee_type_;
        if (robot_arm_index_ == ARM_RIGHT) return right_ee_type_;
        return (tool_idx == 0) ? left_ee_type_ : right_ee_type_;
    }

    bool MarvinHardware::toolIsHand(size_t tool_idx) const
    {
        return eeTypeIsHand(eeTypeForTool(tool_idx));
    }

    bool MarvinHardware::toolUsesLeftChannel(size_t tool_idx) const
    {
        if (tool_idx < tool_is_left_side_.size())
        {
            return tool_is_left_side_[tool_idx];
        }
        return robot_arm_index_ == ARM_LEFT || (robot_arm_index_ == ARM_DUAL && tool_idx == 0);
    }

    long MarvinHardware::toolChannel(size_t tool_idx) const
    {
        return toolUsesLeftChannel(tool_idx) ? left_ee_channel_ : right_ee_channel_;
    }

    void MarvinHardware::contains_tool()
    {
        int joint_index = 0;
        bool should_detect_gripper = !gripper_type_.empty() && gripper_type_ != "NONE";
        
        RCLCPP_INFO(get_logger(), "contains_tool: gripper_type_='%s', should_detect_gripper=%d", 
                    gripper_type_.c_str(), should_detect_gripper);

        // Determine if end effector is a hand or gripper based on type
        // Hand types: LINKERHAND_O7, LINKERHAND_O6, LINKERHAND_L6 (all are LinkerHand/DexterousHand)
        // Also support short forms: O7, O6, L6
        static const std::set<std::string> hand_types = {
            "LINKERHAND_O7", "LINKERHAND_O6", "LINKERHAND_L6",
            "O7", "O6", "L6",
            "FREEDOM_V1", "FREEDOM_V2", "FREEDOM",
            "INSPIRE_E2", "INSPIRE", "RH56E2",
            "INSPIRE_F2", "RH56F2",
            "ERG32"
        };
        // Gripper types: JD, Changingtek variants
        static const std::set<std::string> gripper_types = {
            "RG75", "JDGRIPPER",
            "CHANGINGTEK90C", "AG2F90", "AG2F90C", "AG2F90_C",
            "CHANGINGTEK90D", "AG2F90D", "AG2F90_D",
            "AG2F120S", "AG2F120S_D",
            "EINCINX", "EPGI180"
        };
        
        // Check if type is hand or gripper
        bool is_hand_type = false;
        if (hand_types.find(gripper_type_) != hand_types.end())
        {
            is_hand_type = true;
        }
        else if (gripper_type_.find("LINKERHAND") != std::string::npos ||
                 (gripper_type_.find("HAND") != std::string::npos && 
                  gripper_type_.find("GRIPPER") == std::string::npos))
        {
            // Check for hand type patterns in the string (but not "gripper")
            is_hand_type = true;
        }
        
        if (is_hand_type)
        {
            tool_type_ = ToolType::Hand;
            RCLCPP_INFO(get_logger(), "Detected hand type: %s", gripper_type_.c_str());
        }
        else if (gripper_types.find(gripper_type_) != gripper_types.end())
        {
            tool_type_ = ToolType::Gripper;
            RCLCPP_INFO(get_logger(), "Detected gripper type: %s", gripper_type_.c_str());
        }
        else if (!gripper_type_.empty() && gripper_type_ != "NONE")
        {
            tool_type_ = ToolType::Others;
            RCLCPP_WARN(get_logger(), "Unknown tool type '%s', treating as others", gripper_type_.c_str());
        }
        else
        {
            tool_type_ = ToolType::None;
            RCLCPP_INFO(get_logger(), "No tool type specified");
        }

        RCLCPP_INFO(get_logger(), "Total joints in info_.joints: %zu", info_.joints.size());
        for (const auto& joint : info_.joints)
        {
            bool is_gripper_joint = false;
            
            // 只有在配置了夹爪类型时才检测夹爪关节
            if (should_detect_gripper)
            {
            // 检查关节名称中是否包含 gripper 或 hand
            std::string joint_name_lower = joint.name;
            std::transform(joint_name_lower.begin(), joint_name_lower.end(), 
                        joint_name_lower.begin(), ::tolower);
            
                if (joint_name_lower.find("gripper") != std::string::npos || 
                        joint_name_lower.find("hand") != std::string::npos)
                {
                    // 这是夹爪关节
                    is_gripper_joint = true;
                    has_gripper_ = true;
                    gripper_joint_name_.push_back(joint.name);
                    gripper_joint_index_ = joint_index;
                    RCLCPP_INFO(get_logger(), "Detected %s joint: %s (index %zu)",
                            (tool_type_ == ToolType::Hand) ? "hand" : "gripper", joint.name.c_str(), gripper_joint_index_);
                } 
            }
            
            // 如果不是夹爪关节，则添加到机械臂关节列表
            if (!is_gripper_joint)
            {
                joint_names_.push_back(joint.name);              
            }
            joint_index++;
        }
        RCLCPP_INFO(get_logger(), "Detected %s joints: %zu, arm joints: %zu",
                    (tool_type_ == ToolType::Hand) ? "hand" : "gripper", gripper_joint_name_.size(), joint_names_.size());

        // Print all detected hand/gripper joint names
        if (gripper_joint_name_.size() > 0)
        {
            RCLCPP_INFO(get_logger(), "Detected %s joint names:", (tool_type_ == ToolType::Hand) ? "hand" : "gripper");
            for (size_t i = 0; i < gripper_joint_name_.size(); i++)
            {
                RCLCPP_INFO(get_logger(), "  [%zu] %s", i, gripper_joint_name_[i].c_str());
            }
        }
    }

    hardware_interface::CallbackReturn MarvinHardware::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Activating Marvin Hardware Interface...");

        if (hardware_connected_) {
            // Resume from POWER_OFF: re-engage brake if it was released (via existing brake params)
            if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL) {
                const bool param_released = node_->has_parameter("left_brake_release")
                    && node_->get_parameter("left_brake_release").as_bool();
                if (left_brake_released_ || param_released) {
                    node_->set_parameter(rclcpp::Parameter("left_brake_release", false));
                }
            }
            if (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL) {
                const bool param_released = node_->has_parameter("right_brake_release")
                    && node_->get_parameter("right_brake_release").as_bool();
                if (right_brake_released_ || param_released) {
                    node_->set_parameter(rclcpp::Parameter("right_brake_release", false));
                }
            }

            robot_ctrl_mode_ = last_active_ctrl_mode_;
            int mode = ctrlModeStringToMode(robot_ctrl_mode_, get_logger());
            RCLCPP_INFO(get_logger(), "Already connected, re-enabling arms (resume to %s)", robot_ctrl_mode_.c_str());
            applyRobotConfiguration(mode, -1, -1, -1.0, -1.0,
                                    std::vector<double>(), std::vector<double>(),
                                    std::vector<double>(), std::vector<double>());
            if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL)
                setArmCtrlInternal(ARM_LEFT);
            if (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL)
                setArmCtrlInternal(ARM_RIGHT);
            return hardware_interface::CallbackReturn::SUCCESS;
        }

        // First-time activation: connect to real hardware
        if (!connectToHardware())
        {
            RCLCPP_ERROR(get_logger(), "Failed to connect to Marvin hardware");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Read initial joint states from hardware with validation
        // Try multiple times to ensure data is ready and stable
        const int max_read_attempts = 10;
        const int read_interval_ms = 100;  // 100ms between attempts
        bool initial_read_success = false;
        bool all_zeros_detected = false;
        
        for (int attempt = 0; attempt < max_read_attempts; attempt++)
        {
            if (!readFromHardware(true))
            {
                RCLCPP_WARN(get_logger(), "Failed to read initial joint states (attempt %d/%d)", 
                           attempt + 1, max_read_attempts);
                usleep(read_interval_ms * 1000);
                continue;
            }
            
            // Validate that we didn't get all zeros (which likely indicates uninitialized data)
            bool all_zeros = true;
            for (size_t i = 0; i < hw_position_states_.size(); i++)
            {
                if (std::abs(hw_position_states_[i]) > 1e-6)  // Check if significantly non-zero
                {
                    all_zeros = false;
                    break;
                }
            }
            
            if (all_zeros)
            {
                all_zeros_detected = true;
                RCLCPP_WARN(get_logger(), "Initial joint positions are all zeros (attempt %d/%d), "
                           "this may indicate uninitialized hardware data. Retrying...", 
                           attempt + 1, max_read_attempts);
                usleep(read_interval_ms * 1000);
                continue;
            }
            
            initial_read_success = true;
            RCLCPP_INFO(get_logger(), "Successfully read initial joint states (attempt %d/%d)", 
                       attempt + 1, max_read_attempts);
            break;
        }
        
        if (!initial_read_success)
        {
            RCLCPP_ERROR(get_logger(), "Failed to read initial joint states after %d attempts", 
                        max_read_attempts);
            if (all_zeros_detected)
            {
                RCLCPP_ERROR(get_logger(), "Hardware appears to be returning all zeros. "
                           "This may indicate: 1) Hardware not ready, 2) Communication issue, "
                           "3) Hardware initialization problem. Please check hardware status.");
            }
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Initialize commands with current positions
        for (size_t i = 0; i < hw_position_commands_.size(); i++)
        {
            hw_position_commands_[i] = hw_position_states_[i];
            hw_velocity_commands_[i] = hw_velocity_states_[i];
        }
        
        // Log initial positions for debugging
        // Handle both single arm (7 joints) and dual arm (14 joints) cases
        if (hw_position_states_.size() == 7)
        {
            // Single arm: 7 joints
            RCLCPP_INFO(get_logger(), "Initial joint positions (rad): [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                       hw_position_states_[0], hw_position_states_[1], hw_position_states_[2],
                       hw_position_states_[3], hw_position_states_[4], hw_position_states_[5],
                       hw_position_states_[6]);
        }
        else if (hw_position_states_.size() == 14)
        {
            // Dual arm: 14 joints (left 7 + right 7)
            RCLCPP_INFO(get_logger(), "Initial joint positions (rad) - Left arm: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                       hw_position_states_[0], hw_position_states_[1], hw_position_states_[2],
                       hw_position_states_[3], hw_position_states_[4], hw_position_states_[5],
                       hw_position_states_[6]);
            RCLCPP_INFO(get_logger(), "Initial joint positions (rad) - Right arm: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                       hw_position_states_[7], hw_position_states_[8], hw_position_states_[9],
                       hw_position_states_[10], hw_position_states_[11], hw_position_states_[12],
                       hw_position_states_[13]);
        }
        else
        {
            // Generic case: log all joints
            std::stringstream ss;
            ss << "Initial joint positions (rad): [";
            for (size_t i = 0; i < hw_position_states_.size(); i++)
            {
                ss << std::fixed << std::setprecision(3) << hw_position_states_[i];
                if (i < hw_position_states_.size() - 1)
                {
                    ss << ", ";
                }
            }
            ss << "]";
            RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
        }
        OnClearSet();
        OnLogOff();
        OnLocalLogOff();
        OnSetSend();
        usleep(100000);

        // Initialize gripper parameters (default to all zeros if no gripper)
        // This must be done before setArmCtrlInternal
        leftkineParam_.resize(6, 0.0);
        leftdynParam_.resize(10, 0.0);
        rightkineParam_.resize(6, 0.0);
        rightdynParam_.resize(10, 0.0);
        
        // Connect and initialize end effector (gripper or hand).
        if (init_tool_on_startup_ && has_gripper_ && !gripper_type_.empty() && gripper_type_ != "NONE")
        {
            if (toolCount() == 0)
            {
                RCLCPP_ERROR(get_logger(),
                             "End effector type '%s' configured but no end effector detected or created; "
                             "skipping tool communication",
                             gripper_type_.c_str());
            }
            else if (!connect_tool())
            {
                RCLCPP_ERROR(get_logger(),
                             "%s communication verification failed during initialize(); "
                             "skipping further tool communication for failed side(s)",
                             toolTypeLogName());
            }
        }
        else if (!init_tool_on_startup_ && has_gripper_ && !gripper_type_.empty() && gripper_type_ != "NONE")
        {
            RCLCPP_WARN(get_logger(),
                        "Tool type '%s' is configured but init_tool_on_startup=false, skipping end-effector initialization on startup.",
                        gripper_type_.c_str());
        }

        if (robot_arm_index_ == ARM_LEFT)
        {
            setArmCtrlInternal(ARM_LEFT);
        }
        else if (robot_arm_index_ == ARM_RIGHT)
        {
            setArmCtrlInternal(ARM_RIGHT);
        }
        else if (robot_arm_index_ == ARM_DUAL)
        {
            setArmCtrlInternal(ARM_LEFT);
            setArmCtrlInternal(ARM_RIGHT);
        }

        // Apply robot operation mode from ROS2 parameter ctrl_mode (default is "POSITION")
        // This reads the ctrl_mode parameter and applies the configuration
        // Convert robot_ctrl_mode_ string to integer mode locally
        int mode = -1;
        if (robot_ctrl_mode_ == "POSITION") {
            mode = 1;
        } else if (robot_ctrl_mode_ == "JOINT_IMPEDANCE") {
            mode = 2;
        } else if (robot_ctrl_mode_ == "CART_IMPEDANCE") {
            mode = 3;
        }
        applyRobotConfiguration(mode, -1, -1, -1.0, -1.0, 
                                std::vector<double>(), std::vector<double>(), 
                                std::vector<double>(), std::vector<double>());

        // Read DCSS once at init and log current arm state and error codes
        OnGetBuf(&frame_data_);
        if (robot_arm_index_ == ARM_LEFT)
        {
            RCLCPP_INFO(get_logger(), "DCSS init: Left arm  CurState=%d, CmdState=%d, ERRCode=%d",
                        frame_data_.m_State[0].m_CurState, frame_data_.m_State[0].m_CmdState,
                        frame_data_.m_State[0].m_ERRCode);
            RCLCPP_INFO(get_logger(), "  Left arm vel/acc ratio: %d, %d",
                        frame_data_.m_In[0].m_Joint_Vel_Ratio, frame_data_.m_In[0].m_Joint_Acc_Ratio);
        }
        else if (robot_arm_index_ == ARM_RIGHT)
        {
            RCLCPP_INFO(get_logger(), "DCSS init: Right arm CurState=%d, CmdState=%d, ERRCode=%d",
                        frame_data_.m_State[1].m_CurState, frame_data_.m_State[1].m_CmdState,
                        frame_data_.m_State[1].m_ERRCode);
            RCLCPP_INFO(get_logger(), "  Right arm vel/acc ratio: %d, %d",
                        frame_data_.m_In[1].m_Joint_Vel_Ratio, frame_data_.m_In[1].m_Joint_Acc_Ratio);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "DCSS init: Left arm  CurState=%d, CmdState=%d, ERRCode=%d",
                        frame_data_.m_State[0].m_CurState, frame_data_.m_State[0].m_CmdState,
                        frame_data_.m_State[0].m_ERRCode);
            RCLCPP_INFO(get_logger(), "DCSS init: Right arm CurState=%d, CmdState=%d, ERRCode=%d",
                        frame_data_.m_State[1].m_CurState, frame_data_.m_State[1].m_CmdState,
                        frame_data_.m_State[1].m_ERRCode);
            RCLCPP_INFO(get_logger(), "  Left vel/acc: %d, %d; Right vel/acc: %d, %d",
                        frame_data_.m_In[0].m_Joint_Vel_Ratio, frame_data_.m_In[0].m_Joint_Acc_Ratio,
                        frame_data_.m_In[1].m_Joint_Vel_Ratio, frame_data_.m_In[1].m_Joint_Acc_Ratio);
        }

        // One-time initial read per verified tool; failed tools are marked and never polled again.
        if (init_tool_on_startup_ && has_gripper_ && !gripper_type_.empty() && gripper_type_ != "NONE" && toolCount() > 0)
        {
            bool any_tool_needs_read = false;
            for (size_t ti = 0; ti < toolCount(); ++ti)
            {
                if (!toolAt(ti))
                    continue;
                if (ti < tool_init_failed_.size() && tool_init_failed_[ti])
                    continue;
                any_tool_needs_read = true;
                break;
            }
            if (any_tool_needs_read)
            {
                RCLCPP_INFO(get_logger(), "sleeping 500ms for tool initialization");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                if (!doInitialToolReads())
                {
                    RCLCPP_ERROR(get_logger(),
                                 "%s communication verification failed after initial read; "
                                 "skipping further tool communication for failed side(s)",
                                 toolTypeLogName());
                }
                else
                {
                    RCLCPP_INFO(get_logger(), "Initial end-effector read done; starting tool_callback_for_tool threads");
                }
            }
        }

        if (init_tool_on_startup_ && has_gripper_ && !gripper_type_.empty() && gripper_type_ != "NONE" && toolCount() > 0)
        {
            if (use_async_tool_comm_)
            {
                // Send-only threads; OnGetChData is polled in MarvinHardware::read().
                gripper_ctrl_threads_.resize(toolCount());
                for (size_t ti = 0; ti < toolCount(); ++ti)
                {
                    if (!toolAt(ti)) continue;
                    if (ti < tool_init_failed_.size() && tool_init_failed_[ti]) continue;
                    gripper_ctrl_threads_[ti] = std::thread(&MarvinHardware::tool_callback_for_tool_async, this, ti);
                    gripper_ctrl_threads_[ti].detach();
                }
                bool any_tool_active = false;
                for (size_t ti = 0; ti < toolCount(); ++ti)
                {
                    if (toolAt(ti) && ti < tool_init_failed_.size() && !tool_init_failed_[ti])
                    {
                        any_tool_active = true;
                        break;
                    }
                }
                if (any_tool_active)
                {
                    RCLCPP_INFO(get_logger(),
                                "%s Connected (async send; RS485 get in hardware read)",
                                toolTypeLogName());
                }
            }
            else
            {
                gripper_ctrl_threads_.resize(toolCount());
                for (size_t ti = 0; ti < toolCount(); ++ti)
                {
                    if (!toolAt(ti)) continue;
                    if (ti < tool_init_failed_.size() && tool_init_failed_[ti]) continue;
                    gripper_ctrl_threads_[ti] = std::thread(&MarvinHardware::tool_callback_for_tool, this, ti);
                    gripper_ctrl_threads_[ti].detach();
                }
                bool any_tool_active = false;
                for (size_t ti = 0; ti < toolCount(); ++ti)
                {
                    if (toolAt(ti) && ti < tool_init_failed_.size() && !tool_init_failed_[ti])
                    {
                        any_tool_active = true;
                        break;
                    }
                }
                if (any_tool_active)
                {
                    RCLCPP_INFO(get_logger(), "%s Connected (independent scheduler per tool)", toolTypeLogName());
                }
            }
        }
        else if (!kwr75_ft_config_.enabled)
        {
            if (!init_tool_on_startup_ && !gripper_type_.empty() && gripper_type_ != "NONE")
            {
                RCLCPP_WARN(get_logger(),
                            "Tool type '%s' configured but init_tool_on_startup=false, continuing without tool threads.",
                            gripper_type_.c_str());
            }
            else if (!gripper_type_.empty() && gripper_type_ != "NONE")
            {
                RCLCPP_ERROR(get_logger(),
                             "%s type configured but no verified tool communication; arm will continue without tool",
                             toolTypeLogName());
            }
            else
            {
                RCLCPP_INFO(get_logger(), "No gripper configured, continuing without gripper");
            }
        }

        // KWR75: COM2 get runs in hardware read(); this thread only starts stream + publishes.
        if (kwr75_ft_config_.enabled)
        {
            startKwr75FtThreads();
        }

        RCLCPP_INFO(get_logger(), "Marvin Hardware Interface activated successfully");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void MarvinHardware::tool_callback_for_tool(size_t tool_idx)
    {
        // 固定控制周期循环：
        // - 每个周期：先判断是否需要读取 -> 读取一次（如需要）
        // - 再判断是否需要写 -> 写一次（如需要）
        // - 空闲/停止时不读取（除非还没完成初始读，或末端仍在运动中）
        if (tool_idx >= kMaxTools || tool_idx >= toolCount())
            return;

        constexpr int kPollMs = 10;
        constexpr int kControlPeriodMs = 100;

        while (hardware_connected_)
        {
            const auto cycle_start = std::chrono::steady_clock::now();

            if (!has_gripper_ || toolCount() == 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            auto* tool = toolAt(tool_idx);
            if (!tool)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }
            if (tool_idx < tool_init_failed_.size() && tool_init_failed_[tool_idx])
            {
                std::this_thread::sleep_until(cycle_start + std::chrono::milliseconds(kControlPeriodMs));
                continue;
            }

            // 若有写指令在途未响应，先尝试消费该响应，再决定是否读/写
            if (tool_idx < in_flight_type_.size() && in_flight_type_[tool_idx].load() == 2)
                tryConsumeWriteAck(tool_idx);

            const bool initial_read_done = (tool_idx < tool_initial_read_done_.size() && tool_initial_read_done_[tool_idx]);
            const bool stopped = isToolStopped(tool_idx);
            const bool should_read = !initial_read_done || !stopped;

            if (should_read)
            {
                const bool got = readToolStatusSync(tool_idx, kPollMs);
                accountSyncFrame(tool_idx, 1, got);
            }

            std::vector<double> write_cmd;
            if (shouldSendToolCommand(tool_idx, write_cmd) && !write_cmd.empty())
            {
                const bool got = writeToolStatusSync(tool_idx, write_cmd, kPollMs);
                accountSyncFrame(tool_idx, 2, got);
            }

            emitTimeoutSummary(tool_idx);
            std::this_thread::sleep_until(cycle_start + std::chrono::milliseconds(kControlPeriodMs));
        }
    }

    void MarvinHardware::dispatchToolCom1Frame(size_t tool_idx, const unsigned char* data, long received)
    {
        if (tool_idx >= toolCount() || received < 2 || data == nullptr)
        {
            return;
        }
        if (isModbusWriteAck(data, static_cast<size_t>(received)))
        {
            if (tool_idx < in_flight_type_.size() && in_flight_type_[tool_idx].load() == 2)
            {
                applyGripperWriteAckFromInFlight(tool_idx);
            }
            markFrameAnswered(tool_idx);
            return;
        }
        if (data[1] == 0x03 || data[1] == 0x04)
        {
            const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
            if (tool_idx < tool_hb_last_rx_ms_.size())
            {
                tool_hb_last_rx_ms_[tool_idx].store(static_cast<std::int64_t>(now_ms));
            }
            if (tool_idx < tool_hb_offline_reported_.size())
            {
                tool_hb_offline_reported_[tool_idx].store(false);
            }
            markFrameAnswered(tool_idx);
        }
        processToolResponse(data, static_cast<size_t>(received), tool_idx);
        if (tool_idx < tool_initial_read_done_.size())
        {
            tool_initial_read_done_[tool_idx] = true;
        }
    }

    void MarvinHardware::ingestKwr75Com2Frame(
        bool left_arm, const unsigned char* data, long received, long rx_channel)
    {
        auto& slot = left_arm ? left_kwr75_sample_ : right_kwr75_sample_;
        const long want_ch = left_arm ? kwr75_ft_config_.left_channel : kwr75_ft_config_.right_channel;
        if (!Kwr75Protocol::isExactFrameRead(received, rx_channel, want_ch))
        {
            return;
        }
        std::array<float, Kwr75Protocol::kAxisCount> raw {};
        if (!Kwr75Protocol::tryParseCompleteFrame(
                data, static_cast<std::size_t>(received),
                kwr75_ft_config_.command_code, raw))
        {
            return;
        }
        slot.publish(raw);
    }

    void MarvinHardware::pollRs485InHardwareRead()
    {
        const bool poll_left =
            (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL);
        const bool poll_right =
            (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL);

        auto poll_side = [this](bool left) {
            GetChDataFunc get_ch = left ? MarvinRs485Bus::getA() : MarvinRs485Bus::getB();
            const long com1 = left ? left_ee_channel_ : right_ee_channel_;

            int tool_idx = -1;
            for (size_t ti = 0; ti < toolCount(); ++ti)
            {
                if (!toolAt(ti))
                {
                    continue;
                }
                if (ti < tool_init_failed_.size() && tool_init_failed_[ti])
                {
                    continue;
                }
                if (toolUsesLeftChannel(ti) == left)
                {
                    tool_idx = static_cast<int>(ti);
                    break;
                }
            }

            unsigned char buf[256] = {0};
            if (tool_idx >= 0)
            {
                for (int drain = 0; drain < 4; ++drain)
                {
                    long ch = com1;
                    const long received = get_ch(buf, &ch);
                    if (received <= 0 || received > static_cast<long>(sizeof(buf)))
                    {
                        break;
                    }
                    if (ch == KWR75_FT_CHANNEL)
                    {
                        ingestKwr75Com2Frame(left, buf, received, ch);
                        continue;
                    }
                    if (ch != com1)
                    {
                        continue;
                    }
                    dispatchToolCom1Frame(static_cast<size_t>(tool_idx), buf, received);
                }
            }

            const bool ft_side_on = left ? kwr75_ft_config_.left_enabled
                                         : kwr75_ft_config_.right_enabled;
            if (kwr75_ft_config_.enabled && kwr75_ft_running_.load(std::memory_order_acquire) && ft_side_on)
            {
                long ch = left ? kwr75_ft_config_.left_channel : kwr75_ft_config_.right_channel;
                // 0x49 request/response: when publishing in read(), send one poll before get.
                if (!kwr75UsesPollThread() && kwr75_ft_config_.command_code == 0x49)
                {
                    Send485Func send_485 = left ? MarvinRs485Bus::sendA() : MarvinRs485Bus::sendB();
                    const auto req = Kwr75Protocol::buildPollRequest(kwr75_ft_config_.command_code);
                    send_485(const_cast<uint8_t*>(req.data()), static_cast<long>(req.size()), ch);
                }
                const long received = get_ch(buf, &ch);
                if (received > 0)
                {
                    ingestKwr75Com2Frame(left, buf, received, ch);
                }
            }
        };

        if (poll_left)
        {
            poll_side(true);
        }
        if (poll_right)
        {
            poll_side(false);
        }
    }

    void MarvinHardware::tool_callback_for_tool_async(size_t tool_idx)
    {
        // Async send: only getStatus() and move_gripper/move_hand; no blocking receive (recv thread handles responses).
        constexpr int kControlPeriodMs = 100;
        // Stopped tools still poll at control rate (not 1Hz): shared RS485 often drops most replies.
        constexpr int kStoppedPollIntervalMs = kControlPeriodMs;
        constexpr int kHeartbeatOfflineThresholdMs = 30000;
        if (tool_idx >= kMaxTools || tool_idx >= toolCount())
            return;

        while (hardware_connected_)
        {
            const auto cycle_start = std::chrono::steady_clock::now();
            const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                cycle_start.time_since_epoch()).count();

            // At the start of each cycle: account for the previous cycle's frame.
            // If still pending, the recv thread never observed a reply -> timeout.
            checkPrevCycleTimeout(tool_idx);
            emitTimeoutSummary(tool_idx);

            if (!has_gripper_ || toolCount() == 0)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            auto* tool = toolAt(tool_idx);
            if (!tool)
            {
                std::this_thread::sleep_until(cycle_start + std::chrono::milliseconds(kControlPeriodMs));
                continue;
            }
            if (tool_idx < tool_init_failed_.size() && tool_init_failed_[tool_idx])
            {
                std::this_thread::sleep_until(cycle_start + std::chrono::milliseconds(kControlPeriodMs));
                continue;
            }

            const bool initial_read_done = (tool_idx < tool_initial_read_done_.size() && tool_initial_read_done_[tool_idx]);
            const bool stopped = isToolStopped(tool_idx);
            const bool should_read = !initial_read_done || !stopped;

            std::vector<double> write_cmd;
            const bool should_write = shouldSendToolCommand(tool_idx, write_cmd) && !write_cmd.empty();

            // Heartbeat: consider link healthy if we've parsed any valid status response recently.
            if (tool_idx < tool_hb_start_ms_.size() && tool_idx < tool_hb_last_rx_ms_.size() && tool_idx < tool_hb_offline_reported_.size())
            {
                auto start_ms = tool_hb_start_ms_[tool_idx].load();
                if (start_ms <= 0)
                {
                    tool_hb_start_ms_[tool_idx].store(static_cast<std::int64_t>(now_ms));
                    start_ms = static_cast<std::int64_t>(now_ms);
                }
                const auto last_rx_ms = tool_hb_last_rx_ms_[tool_idx].load();
                const auto age_ms = (last_rx_ms > 0) ? (now_ms - last_rx_ms) : (now_ms - start_ms);
                if (age_ms >= kHeartbeatOfflineThresholdMs && !tool_hb_offline_reported_[tool_idx].load())
                {
                    tool_hb_offline_reported_[tool_idx].store(true);
                    RCLCPP_ERROR(get_logger(),
                                 "tool heartbeat offline: tool_idx=%zu 超过%.1fs未收到有效状态应答",
                                 tool_idx, static_cast<double>(age_ms) / 1000.0);
                    if (hardware_error_pub_)
                    {
                        std_msgs::msg::Int64 msg;
                        msg.data = toolUsesLeftChannel(tool_idx) ? 0x410001 : 0x420001;
                        hardware_error_pub_->publish(msg);
                        RCLCPP_ERROR(get_logger(),
                                     "published /Base_HardwareError: tool_idx=%zu code=0x%llX",
                                     tool_idx, static_cast<long long>(msg.data));
                    }
                }
            }

            // 一个控制周期内只发一条 set：优先控制指令，其次读状态
            if (should_write)
            {
                if (toolIsHand(tool_idx))
                {
                    auto* hand = dynamic_cast<ModbusHand*>(tool);
                    if (hand && write_cmd.size() >= hand->getJointCount())
                    {
                        const size_t dof = hand->getJointCount();
                        std::vector<double> torques(dof, 1.0);
                        std::vector<double> velocities(dof, 1.0);
                        for (size_t k = 0; k < gripper_joint_name_.size() && k < gripper_effort_command_.size() && k < gripper_velocity_command_.size(); ++k)
                        {
                            if (!gripperJointBelongsToTool(k, tool_idx)) continue;
                            int li = hand->mapJointNameToIndex(gripper_joint_name_[k]);
                            if (li >= 0 && static_cast<size_t>(li) < dof)
                            {
                                torques[static_cast<size_t>(li)] = std::clamp(gripper_effort_command_[k], 0.0, 1.0);
                                velocities[static_cast<size_t>(li)] = std::clamp(gripper_velocity_command_[k], 0.0, 1.0);
                            }
                        }
                        std::vector<double> pos(write_cmd.begin(), write_cmd.begin() + static_cast<std::vector<double>::difference_type>(dof));
                        hand->move_hand(torques, velocities, pos);
                        markFrameSent(tool_idx, 2);
                        for (size_t k = 0; k < gripper_joint_name_.size() && k < last_gripper_command_.size() && k < gripper_stopped_.size(); ++k)
                        {
                            if (!gripperJointBelongsToTool(k, tool_idx)) continue;
                            int li = hand->mapJointNameToIndex(gripper_joint_name_[k]);
                            if (li >= 0 && static_cast<size_t>(li) < dof)
                            {
                                last_gripper_command_[k] = write_cmd[static_cast<size_t>(li)];
                                gripper_stopped_[k] = false;
                                if (k < last_gripper_effort_ack_.size() && k < gripper_effort_command_.size())
                                    last_gripper_effort_ack_[k] = gripper_effort_command_[k];
                                if (k < last_gripper_velocity_ack_.size() && k < gripper_velocity_command_.size())
                                    last_gripper_velocity_ack_[k] = gripper_velocity_command_[k];
                            }
                        }
                        if (tool_idx < hand_stabilized_.size() && tool_idx < hand_stable_count_.size())
                        {
                            hand_stabilized_[tool_idx] = false;
                            hand_stable_count_[tool_idx] = 0;
                        }
                    }
                }
                else
                {
                    const size_t gi = gripperJointIndexForTool(tool_idx);
                    const double nt = (gi < gripper_effort_command_.size())
                        ? std::clamp(gripper_effort_command_[gi], 0.0, 1.0) : 1.0;
                    const double nv = (gi < gripper_velocity_command_.size())
                        ? std::clamp(gripper_velocity_command_[gi], 0.0, 1.0) : 1.0;
                    if (tool->move_gripper(nt, nv, write_cmd[0]))
                    {
                        if (tool_idx < in_flight_write_command_.size())
                        {
                            in_flight_write_command_[tool_idx] = write_cmd;
                            applyGripperWriteAckFromInFlight(tool_idx);
                        }
                        // 异步模式：不等待控制指令返回帧，只根据读状态返回帧更新状态
                        markFrameSent(tool_idx, 2);
                    }
                }
            }
            else
            {
                // Priority: normal status read first; when stopped, keep polling at control rate for heartbeat.
                if (should_read)
                {
                    tool->getStatus();
                    if (tool_idx < tool_hb_tx_ms_.size())
                        tool_hb_tx_ms_[tool_idx].store(static_cast<std::int64_t>(now_ms));
                    markFrameSent(tool_idx, 1);
                }
                else if (tool_idx < tool_hb_tx_ms_.size())
                {
                    const auto last_tx = tool_hb_tx_ms_[tool_idx].load();
                    const bool interval_due = (last_tx <= 0) ||
                        ((now_ms - last_tx) >= kStoppedPollIntervalMs);
                    if (interval_due)
                    {
                        tool->getStatus();
                        tool_hb_tx_ms_[tool_idx].store(static_cast<std::int64_t>(now_ms));
                        markFrameSent(tool_idx, 1);
                    }
                }
            }

            std::this_thread::sleep_until(cycle_start + std::chrono::milliseconds(kControlPeriodMs));
        }
    }

    bool MarvinHardware::connect_tool()
    {
        bool result = true;
        for (size_t i = 0; i < toolCount(); i++)
        {
            if (i < tool_init_failed_.size())
                tool_init_failed_[i] = false;

            if (auto* tool = toolAt(i))
            {
                // Clear this tool's channel receive buffer BEFORE initialize() sends the
                // enable/version commands, so the gripper's early replies are not discarded.
                if (robot_arm_index_ == ARM_RIGHT)
                    MarvinRs485Bus::clearB()();
                else if (robot_arm_index_ == ARM_LEFT)
                    MarvinRs485Bus::clearA()();
                else if (i == 0)
                    MarvinRs485Bus::clearA()();
                else if (i == 1)
                    MarvinRs485Bus::clearB()();

                if (auto* mg = dynamic_cast<ModbusGripper*>(tool))
                {
                    const size_t gi = gripperJointIndexForTool(i);
                    const double nt = (gi < gripper_effort_command_.size())
                                            ? std::clamp(gripper_effort_command_[gi], 0.0, 1.0)
                                            : 1.0;
                    const double nv = (gi < gripper_velocity_command_.size())
                                            ? std::clamp(gripper_velocity_command_[gi], 0.0, 1.0)
                                            : 1.0;
                    mg->setInitialToolConfig(nt, nv);
                }

                const bool ok = tool->initialize();
                if (!ok)
                {
                    if (i < tool_init_failed_.size())
                        tool_init_failed_[i] = true;
                    RCLCPP_ERROR(get_logger(),
                                 "Tool initialize failed: tool_idx=%zu type=%s channel=%ld",
                                 i, eeTypeForTool(i).c_str(), toolChannel(i));
                    result = false;
                    continue;
                }
            }
        }
        return result;
    }

    bool MarvinHardware::doInitialToolReads()
    {
        const size_t n = toolCount();
        constexpr int kMaxRetries = 3;
        constexpr int kFirstWaitMs = 100;
        constexpr int kRetryWaitMs = 100;
        bool all_ok = true;
        for (size_t ti = 0; ti < n; ++ti)
        {
            if (!toolAt(ti))
                continue;
            if (ti < tool_init_failed_.size() && tool_init_failed_[ti])
            {
                all_ok = false;
                continue;
            }
            if (toolUsesLeftChannel(ti))
            {
                MarvinRs485Bus::clearA()();
            }
            else
            {
                MarvinRs485Bus::clearB()();
            }
            bool ok = false;
            for (int attempt = 0; attempt < kMaxRetries && !ok; ++attempt)
            {
                if (attempt > 0)
                {
                    RCLCPP_INFO(get_logger(), "Retry %d/%d for tool_idx=%zu", attempt + 1, kMaxRetries, ti);
                    std::this_thread::sleep_for(std::chrono::milliseconds(kRetryWaitMs));
                }
                const bool got_frame = readToolStatusSync(ti, attempt == 0 ? kFirstWaitMs : 50);
                if (!got_frame)
                    continue;
                if (ti < tool_has_valid_state_.size() && tool_has_valid_state_[ti].load())
                    ok = true;
            }
            if (!ok)
            {
                if (ti < tool_init_failed_.size())
                    tool_init_failed_[ti] = true;
                if (ti < tool_initial_read_done_.size())
                    tool_initial_read_done_[ti] = false;
                RCLCPP_ERROR(get_logger(),
                             "Initial tool read failed for tool_idx=%zu after %d attempts (no valid status); "
                             "skipping further communication for this tool",
                             ti, kMaxRetries);
                all_ok = false;
                continue;
            }
            if (ti < tool_initial_read_done_.size())
                tool_initial_read_done_[ti] = true;
            auto* tool = toolAt(ti);
            auto* gripper = dynamic_cast<ModbusGripper*>(tool);
            if (gripper && gripper->isTargetReached())
            {
                size_t gi = gripperJointIndexForTool(ti);
                if (gi < gripper_stopped_.size())
                    gripper_stopped_[gi] = true;
            }

            if (toolIsHand(ti))
            {
                const size_t n_j = std::min(gripper_joint_name_.size(),
                                            std::min(gripper_position_.size(),
                                                     std::min(gripper_position_command_.size(), last_gripper_command_.size())));
                std::vector<double> current;
                current.reserve(8);
                for (size_t gi = 0; gi < n_j; ++gi)
                {
                    if (!gripperJointBelongsToTool(gi, ti))
                        continue;
                    const double pos = gripper_position_[gi];
                    gripper_position_command_[gi] = pos;
                    last_gripper_command_[gi] = pos;
                    if (gi < gripper_stopped_.size())
                        gripper_stopped_[gi] = true;
                    current.push_back(pos);
                }
                if (ti < hand_previous_position_.size() && ti < hand_stable_count_.size() && ti < hand_stabilized_.size())
                {
                    hand_previous_position_[ti] = current;
                    hand_stable_count_[ti] = kHandStableFrameCount;
                    hand_stabilized_[ti] = true;
                }
                syncHandDynamicsAckToCommand(ti);
            }
            else
            {
                const size_t gi = gripperJointIndexForTool(ti);
                if (gi < gripper_position_.size() && gi < gripper_position_command_.size() && gi < last_gripper_command_.size())
                {
                    const double pos = gripper_position_[gi];
                    gripper_position_command_[gi] = pos;
                    last_gripper_command_[gi] = pos;
                    if (gi < gripper_stopped_.size())
                        gripper_stopped_[gi] = true;
                    if (gi < gripper_previous_position_.size())
                        gripper_previous_position_[gi] = pos;
                    if (gi < gripper_stable_count_.size())
                        gripper_stable_count_[gi] = kGripperStableFrameCount;
                }
            }
            RCLCPP_INFO(get_logger(), "Initial tool read done for tool_idx=%zu", ti);
        }
        return all_ok;
    }

    bool MarvinHardware::readToolStatusSync(size_t tool_idx, int elapsed_time_for_poll)
    {
        if (tool_idx >= toolCount())
            return false;
        if (tool_idx < tool_init_failed_.size() && tool_init_failed_[tool_idx])
            return false;
        auto* tool = toolAt(tool_idx);
        if (!tool)
            return false;
        GetChDataFunc get_ch = toolUsesLeftChannel(tool_idx)
            ? MarvinRs485Bus::getA() : MarvinRs485Bus::getB();
        tool->getStatus();

        // Windowed receive: poll the channel repeatedly within an 80ms window instead
        // of a single shot. The gripper's reply can arrive tens of ms after the send,
        // and a single get_ch_data after a fixed sleep frequently misses it (the root
        // cause of the original "received no bytes" failures).
        constexpr int kRecvWindowMs = 80;
        constexpr int kRecvPollIntervalMs = 5;
        // Honor a small initial turnaround (RS485 turn-around) before first poll.
        std::this_thread::sleep_for(std::chrono::milliseconds(
            std::min(elapsed_time_for_poll > 0 ? elapsed_time_for_poll : kRecvPollIntervalMs, 10)));
        const auto deadline = std::chrono::steady_clock::now()
            + std::chrono::milliseconds(kRecvWindowMs);

        unsigned char data_buf[256] = {0};
        long received = 0;
        bool got_valid_status = false;
        while (true)
        {
            received = receiveToolResponse(data_buf, sizeof(data_buf), get_ch, toolChannel(tool_idx));
            if (received > 0)
            {
                if (isModbusWriteAck(data_buf, static_cast<size_t>(received)))
                {
                    applyGripperWriteAckFromInFlight(tool_idx);
                    // Stale FC06/FC10 ack from init writes — keep polling for FC03 status.
                }
                else
                {
                    processToolResponse(data_buf, static_cast<size_t>(received), tool_idx);
                    if (tool_idx < tool_has_valid_state_.size() &&
                        tool_has_valid_state_[tool_idx].load())
                    {
                        got_valid_status = true;
                        break;
                    }
                }
            }
            if (std::chrono::steady_clock::now() >= deadline)
            {
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(kRecvPollIntervalMs));
        }

        if (!got_valid_status)
        {
            if (received <= 0)
            {
                RCLCPP_WARN(get_logger(),
                            "readToolStatusSync(tool_idx=%zu, channel=%ld) received no bytes within %dms window",
                            tool_idx, toolChannel(tool_idx), kRecvWindowMs);
            }
            return false;
        }
        if (tool_idx < tool_initial_read_done_.size())
        {
            tool_initial_read_done_[tool_idx] = true;
        }
        return true;
    }

    void MarvinHardware::syncHandDynamicsAckToCommand(size_t tool_idx)
    {
        if (tool_idx >= toolCount())
            return;
        auto* hand = dynamic_cast<marvin_ros2_control::ModbusHand*>(toolAt(tool_idx));
        if (!hand)
            return;

        const size_t tool_n = toolCount();
        const size_t n = std::min(gripper_joint_name_.size(),
                                  std::min(gripper_effort_command_.size(),
                                           gripper_velocity_command_.size()));
        const size_t dof = hand->getJointCount();
        for (size_t gi = 0; gi < n; ++gi)
        {
            std::string name_lower = gripper_joint_name_[gi];
            std::transform(name_lower.begin(), name_lower.end(), name_lower.begin(), ::tolower);
            const size_t mapped_tool = (tool_n >= 2 && name_lower.find("right_hand_") != std::string::npos) ? 1 : 0;
            if (mapped_tool != tool_idx)
                continue;

            const int li = hand->mapJointNameToIndex(gripper_joint_name_[gi]);
            if (li < 0 || static_cast<size_t>(li) >= dof)
                continue;

            if (gi < last_gripper_velocity_ack_.size())
                last_gripper_velocity_ack_[gi] = gripper_velocity_command_[gi];
            if (gi < last_gripper_effort_ack_.size())
                last_gripper_effort_ack_[gi] = gripper_effort_command_[gi];
        }
    }

    bool MarvinHardware::writeToolStatusSync(size_t tool_idx, const std::vector<double>& write_cmd, int wait_ms)
    {
        if (tool_idx >= toolCount() || write_cmd.empty())
            return false;
        if (tool_idx < tool_init_failed_.size() && tool_init_failed_[tool_idx])
            return false;
        auto* tool = toolAt(tool_idx);
        if (!tool)
            return false;
        GetChDataFunc get_ch = toolUsesLeftChannel(tool_idx)
            ? MarvinRs485Bus::getA() : MarvinRs485Bus::getB();
        if (toolIsHand(tool_idx))
        {
            auto* hand = dynamic_cast<marvin_ros2_control::ModbusHand*>(tool);
            if (!hand || write_cmd.size() < hand->getJointCount())
                return false;
            const size_t dof = hand->getJointCount();
            std::vector<double> torques(dof, 1.0);
            std::vector<double> velocities(dof, 1.0);
            for (size_t k = 0; k < gripper_joint_name_.size() && k < gripper_effort_command_.size() && k < gripper_velocity_command_.size(); ++k)
            {
                if (!gripperJointBelongsToTool(k, tool_idx)) continue;
                int li = hand->mapJointNameToIndex(gripper_joint_name_[k]);
                if (li >= 0 && static_cast<size_t>(li) < dof)
                {
                    torques[static_cast<size_t>(li)] = std::clamp(gripper_effort_command_[k], 0.0, 1.0);
                    velocities[static_cast<size_t>(li)] = std::clamp(gripper_velocity_command_[k], 0.0, 1.0);
                }
            }
            std::vector<double> pos(write_cmd.begin(), write_cmd.begin() + static_cast<std::vector<double>::difference_type>(dof));
            hand->move_hand(torques, velocities, pos);
        }
        else
        {
            const size_t gi = gripperJointIndexForTool(tool_idx);
            const double nt = (gi < gripper_effort_command_.size())
                ? std::clamp(gripper_effort_command_[gi], 0.0, 1.0) : 1.0;
            const double nv = (gi < gripper_velocity_command_.size())
                ? std::clamp(gripper_velocity_command_[gi], 0.0, 1.0) : 1.0;
            if (!tool->move_gripper(nt, nv, write_cmd[0]))
                return false;
            if (tool_idx < in_flight_type_.size() && tool_idx < in_flight_write_command_.size())
            {
                in_flight_type_[tool_idx].store(2);
                in_flight_write_command_[tool_idx] = write_cmd;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(wait_ms));
        unsigned char data_buf[256] = {0};
        long received = receiveToolResponse(data_buf, sizeof(data_buf), get_ch, toolChannel(tool_idx));
        if (received <= 0 || !isModbusWriteAck(data_buf, static_cast<size_t>(received)))
            return false;
        if (!toolIsHand(tool_idx) && tool_idx < in_flight_type_.size())
            in_flight_type_[tool_idx].store(0);
        size_t gi = gripperJointIndexForTool(tool_idx);
        if (toolIsHand(tool_idx))
        {
            auto* hand = dynamic_cast<marvin_ros2_control::ModbusHand*>(tool);
            if (hand)
            {
                const size_t dof = hand->getJointCount();
                for (size_t k = 0; k < gripper_joint_name_.size() && k < last_gripper_command_.size() && k < gripper_stopped_.size(); ++k)
                {
                    if (!gripperJointBelongsToTool(k, tool_idx)) continue;
                    int li = hand->mapJointNameToIndex(gripper_joint_name_[k]);
                    if (li >= 0 && static_cast<size_t>(li) < dof)
                    {
                        last_gripper_command_[k] = write_cmd[li];
                        gripper_stopped_[k] = false;
                    }
                }
                if (tool_idx < hand_stabilized_.size() && tool_idx < hand_stable_count_.size())
                {
                    hand_stabilized_[tool_idx] = false;
                    hand_stable_count_[tool_idx] = 0;
                }
            }
        }
        else if (gi < last_gripper_command_.size() && !write_cmd.empty())
        {
            last_gripper_command_[gi] = write_cmd[0];
            if (gi < gripper_stopped_.size())
                gripper_stopped_[gi] = false;
            if (gi < gripper_stable_count_.size())
                gripper_stable_count_[gi] = 0;
            if (gi < gripper_previous_position_.size() && gi < gripper_position_.size())
                gripper_previous_position_[gi] = gripper_position_[gi];
        }
        return true;
    }

    hardware_interface::CallbackReturn MarvinHardware::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Deactivating Marvin Hardware Interface (power off, keep connection)...");

        stopKwr75FtThreads();

        if (robot_arm_index_ == ARM_LEFT)
        {
            OnClearSet();
            OnSetDragSpace_A(0);
            OnSetTargetState_A(0);
            OnSetSend();
            usleep(100000);
        }
        else if (robot_arm_index_ == ARM_RIGHT)
        {
            OnClearSet();
            OnSetTargetState_B(0);
            OnSetSend();
            usleep(100000);
        }
        else if (robot_arm_index_ == ARM_DUAL)
        {
            OnClearSet();
            OnSetTargetState_A(0);
            OnSetSend();
            usleep(100000);
            OnClearSet();
            OnSetTargetState_B(0);
            OnSetSend();
            usleep(100000);
        }

        robot_ctrl_mode_ = "POWER_OFF";

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MarvinHardware::on_shutdown(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Shutting down Marvin Hardware Interface...");

        stopKwr75FtThreads();

        if (hardware_connected_) {
            const bool update_left = (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL);
            const bool update_right = (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL);
            const auto ensure_brake = [this](int arm_index) {
                char name[30] = {};
                std::snprintf(name, sizeof(name), "BRAK%d", arm_index);
                long value = 0;
                OnGetIntPara(name, &value);
                if (value != 1) {
                    OnClearSet();
                    OnSetIntPara(name, 1);
                    OnSetSend();
                    usleep(100000);
                    RCLCPP_WARN(get_logger(), "Emergency brake engaged: %s", name);
                }
            };
            if (update_left) ensure_brake(ARM_LEFT);
            if (update_right) ensure_brake(ARM_RIGHT);

            if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL) {
                OnClearSet();
                OnSetTargetState_A(0);
                OnSetSend();
                usleep(100000);
            }
            if (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL) {
                OnClearSet();
                OnSetTargetState_B(0);
                OnSetSend();
                usleep(100000);
            }

            disconnectFromHardware();
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MarvinHardware::on_error(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_ERROR(get_logger(), "Error in Marvin Hardware Interface, emergency brake...");

        stopKwr75FtThreads();
        robot_ctrl_mode_ = "POWER_OFF";

        if (hardware_connected_) {
            const bool update_left = (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL);
            const bool update_right = (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL);
            const auto ensure_brake = [this](int arm_index) {
                char name[30] = {};
                std::snprintf(name, sizeof(name), "BRAK%d", arm_index);
                long value = 0;
                OnGetIntPara(name, &value);
                if (value != 1) {
                    OnClearSet();
                    OnSetIntPara(name, 1);
                    OnSetSend();
                    usleep(100000);
                    RCLCPP_WARN(get_logger(), "Emergency brake engaged: %s", name);
                }
            };
            if (update_left) ensure_brake(ARM_LEFT);
            if (update_right) ensure_brake(ARM_RIGHT);

            if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL) {
                OnClearSet();
                OnSetTargetState_A(0);
                OnSetSend();
                usleep(100000);
            }
            if (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL) {
                OnClearSet();
                OnSetTargetState_B(0);
                OnSetSend();
                usleep(100000);
            }

            disconnectFromHardware();
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface::ConstSharedPtr> MarvinHardware::on_export_state_interfaces()
    {
        RCLCPP_DEBUG(get_logger(), "Exporting state interfaces for %zu joints",
                     info_.joints.size());
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
        size_t joint_name_sz = joint_names_.size();
        RCLCPP_INFO(get_logger(), "Exporting state interfaces for %zu joints", joint_name_sz);

        for (size_t i = 0; i < joint_name_sz; i++)
        {
            state_interfaces.push_back(
                std::make_shared<hardware_interface::StateInterface>(
                    joint_names_[i],
                    hardware_interface::HW_IF_POSITION,
                    &hw_position_states_[i]));

            state_interfaces.push_back(
                std::make_shared<hardware_interface::StateInterface>(
                    joint_names_[i],
                    hardware_interface::HW_IF_VELOCITY,
                    &hw_velocity_states_[i]));

            state_interfaces.push_back(
                std::make_shared<hardware_interface::StateInterface>(
                    joint_names_[i],
                    hardware_interface::HW_IF_EFFORT,
                    &hw_effort_states_[i]));
        }
        if (has_gripper_)
        {
            for (size_t i = 0; i < gripper_joint_name_.size(); i++)
            {
                state_interfaces.push_back(
                    std::make_shared<hardware_interface::StateInterface>(
                        gripper_joint_name_[i],
                        hardware_interface::HW_IF_POSITION,
                        &gripper_position_[i]));
                if (i < gripper_velocity_.size())
                {
                    state_interfaces.push_back(
                        std::make_shared<hardware_interface::StateInterface>(
                            gripper_joint_name_[i],
                            hardware_interface::HW_IF_VELOCITY,
                            &gripper_velocity_[i]));
                }
                if (i < gripper_effort_.size())
                {
                    state_interfaces.push_back(
                        std::make_shared<hardware_interface::StateInterface>(
                            gripper_joint_name_[i],
                            hardware_interface::HW_IF_EFFORT,
                            &gripper_effort_[i]));
                }
            }
        }

        // Export virtual FT sensor state interfaces for admittance_controller.
        // NOTE:
        // Some setups may not populate info_.sensors reliably even though URDF declares <sensor>.
        // To avoid startup mismatch, we export a deterministic set based on arm configuration.
        const auto add_ft_sensor_interfaces = [&state_interfaces](
            const std::string& sensor_name, std::array<double, 6>& data)
        {
            static const std::array<const char*, 6> kNames = {
                "force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"};
            for (size_t i = 0; i < kNames.size(); ++i)
            {
                state_interfaces.push_back(
                    std::make_shared<hardware_interface::StateInterface>(
                        sensor_name, kNames[i], &data[i]));
            }
        };

        if (robot_arm_index_ == ARM_DUAL)
        {
            add_ft_sensor_interfaces("left_ft_sensor", left_ft_state_);
            add_ft_sensor_interfaces("right_ft_sensor", right_ft_state_);
        }
        else
        {
            add_ft_sensor_interfaces("ft_sensor", single_ft_state_);
        }
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface::SharedPtr> MarvinHardware::on_export_command_interfaces()
    {
        RCLCPP_DEBUG(get_logger(), "Exporting command interfaces for %zu joints",
                     info_.joints.size());

        std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
        size_t joint_name_sz = joint_names_.size();
        RCLCPP_INFO(get_logger(), "Exporting command interfaces for %zu joints",
                    joint_name_sz);
        for (size_t i = 0; i < joint_name_sz; i++)
        {
            command_interfaces.push_back(
                std::make_shared<hardware_interface::CommandInterface>(
                    joint_names_[i],
                    hardware_interface::HW_IF_POSITION,
                    &hw_position_commands_[i]));

            command_interfaces.push_back(
                std::make_shared<hardware_interface::CommandInterface>(
                    joint_names_[i],
                    hardware_interface::HW_IF_VELOCITY,
                    &hw_velocity_commands_[i]));
        }

        if (has_gripper_)
        {
            for (size_t i = 0; i < gripper_joint_name_.size(); i++)
            {
                command_interfaces.push_back(
                    std::make_shared<hardware_interface::CommandInterface>(
                        gripper_joint_name_[i],
                        hardware_interface::HW_IF_POSITION,
                        &gripper_position_command_[i]));
            }
        }

        return command_interfaces;
    }

    hardware_interface::return_type MarvinHardware::read(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        if (!hardware_connected_)
        {
            return hardware_interface::return_type::ERROR;
        }

        if (!readFromHardware(false))
        {
            return hardware_interface::return_type::ERROR;
        }

        // Single OnGetChData entry: COM1 tools + COM2 KWR75 (ros2_control update rate).
        if (use_async_tool_comm_ || kwr75_ft_config_.enabled)
        {
            pollRs485InHardwareRead();
        }
        // Stream mode default: publish at control rate; poll thread only when interval>0.
        if (kwr75_ft_config_.enabled && !kwr75UsesPollThread())
        {
            publishKwr75SyncedToControl();
        }

        // For single-arm setups, mirror the corresponding side into ft_sensor.
        // This keeps the semantic-component interfaces stable regardless of arm configuration.
        {
            std::lock_guard<std::mutex> lock(wrench_mutex_);
            if (robot_arm_index_ == ARM_LEFT) {
                single_ft_state_ = left_ft_state_;
            } else if (robot_arm_index_ == ARM_RIGHT) {
                single_ft_state_ = right_ft_state_;
            }
        }

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type MarvinHardware::write(
        const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
    {
        if (!hardware_connected_)
        {
            return hardware_interface::return_type::ERROR;
        }
        /// convert rad to degree (use pre-allocated buffer to avoid allocations in control loop)
        if (hw_commands_deg_buffer_.size() != hw_position_commands_.size())
        {
            hw_commands_deg_buffer_.resize(hw_position_commands_.size(), 0.0);
        }
        for (size_t i = 0; i < hw_position_commands_.size(); i++)
        {
            hw_commands_deg_buffer_[i] = radToDegree(hw_position_commands_[i]);
        }


        if (!writeToHardware(hw_commands_deg_buffer_))
        {
            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
    }

    /*
        更新夹爪的状态 函数到 ros2 control指定的变量 中
    */
    void MarvinHardware::updateGripperState(size_t gripper_idx, double position, int velocity, int torque)
    {
        if (gripper_idx >= gripper_position_.size() || gripper_idx >= gripper_velocity_.size() ||
            gripper_idx >= gripper_effort_.size())
            return;

        gripper_position_[gripper_idx] = position;
        gripper_velocity_[gripper_idx] = static_cast<double>(velocity);
        gripper_effort_[gripper_idx] = static_cast<double>(torque);
        if (gripper_idx < gripper_previous_position_.size() && gripper_idx < gripper_stable_count_.size() &&
            gripper_idx < gripper_stopped_.size())
        {
            const double prev = gripper_previous_position_[gripper_idx];
            if (!std::isfinite(prev))
            {
                // Baseline not ready (e.g. right after a new command): establish baseline and treat as moving.
                gripper_previous_position_[gripper_idx] = position;
                gripper_stable_count_[gripper_idx] = 0;
                gripper_stopped_[gripper_idx] = false;
            }
            else
            {
                const bool no_diff = (std::abs(position - prev) <= kGripperStableEpsilon);
                if (no_diff)
                    gripper_stable_count_[gripper_idx]++;
                else
                    gripper_stable_count_[gripper_idx] = 0;
                gripper_previous_position_[gripper_idx] = position;

                gripper_stopped_[gripper_idx] = (gripper_stable_count_[gripper_idx] >= kGripperStableFrameCount);
            }
        }
    }

    bool MarvinHardware::isToolStateCloseToCommand(size_t tool_idx, double threshold)
    {
        if (!has_gripper_ || toolCount() == 0)
            return true;
        if (toolIsHand(tool_idx))
        {
            const size_t n = std::min(gripper_joint_name_.size(),
                                     std::min(gripper_position_.size(), gripper_position_command_.size()));
            for (size_t gi = 0; gi < n; ++gi)
            {
                if (!gripperJointBelongsToTool(gi, tool_idx)) continue;
                if (std::abs(gripper_position_[gi] - gripper_position_command_[gi]) > threshold)
                    return false;
            }
            return true;
        }
        size_t gi = gripperJointIndexForTool(tool_idx);
        if (gi >= gripper_position_.size() || gi >= gripper_position_command_.size())
            return true;
        return std::abs(gripper_position_[gi] - gripper_position_command_[gi]) <= threshold;
    }

    size_t MarvinHardware::gripperJointIndexForTool(size_t tool_idx) const
    {
        for (size_t k = 0; k < gripper_joint_name_.size(); ++k)
        {
            if (mappedToolIndexForJoint(k) == tool_idx)
            {
                return k;
            }
        }
        return tool_idx < gripper_joint_name_.size() ? tool_idx : 0;
    }

    bool MarvinHardware::shouldSendToolCommand(size_t tool_idx, std::vector<double>& write_cmd_out)
    {
        write_cmd_out.clear();
        if (tool_idx >= toolCount())
            return false;
        if (tool_idx < tool_init_failed_.size() && tool_init_failed_[tool_idx])
            return false;
        auto* tool = toolAt(tool_idx);
        if (!tool)
            return false;
        if (toolIsHand(tool_idx))
        {
            auto* hand = dynamic_cast<marvin_ros2_control::ModbusHand*>(tool);
            if (!hand)
                return false;
            const size_t n = std::min(gripper_joint_name_.size(), gripper_position_command_.size());
            const size_t dof = hand->getJointCount();
            write_cmd_out.resize(dof, 0.0);
            bool any_changed = false;
            for (size_t gi = 0; gi < n; ++gi)
            {
                if (!gripperJointBelongsToTool(gi, tool_idx))
                    continue;
                int li = hand->mapJointNameToIndex(gripper_joint_name_[gi]);
                if (li >= 0 && static_cast<size_t>(li) < dof)
                {
                    write_cmd_out[li] = gripper_position_command_[gi];
                    if (gi < last_gripper_command_.size() &&
                        CommandChangeDetector::hasChanged(last_gripper_command_[gi], gripper_position_command_[gi], 0.001))
                        any_changed = true;
                    if (gi < gripper_effort_command_.size() && gi < last_gripper_effort_ack_.size() &&
                        (std::isnan(last_gripper_effort_ack_[gi]) ||
                         CommandChangeDetector::hasChanged(last_gripper_effort_ack_[gi], gripper_effort_command_[gi], 0.001)))
                        any_changed = true;
                    if (gi < gripper_velocity_command_.size() && gi < last_gripper_velocity_ack_.size() &&
                        (std::isnan(last_gripper_velocity_ack_[gi]) ||
                         CommandChangeDetector::hasChanged(last_gripper_velocity_ack_[gi], gripper_velocity_command_[gi], 0.001)))
                        any_changed = true;
                }
            }
            if (any_changed)
            {
                RCLCPP_DEBUG(get_logger(), "shouldSendToolCommand: hand write tool_idx=%zu", tool_idx);
            }
            return any_changed;
        }
        size_t gi = gripperJointIndexForTool(tool_idx); // gripper index 
        if (gi >= last_gripper_command_.size() || gi >= gripper_position_command_.size())
            return false;
        // 已有写指令在途未响应时不再发新写，直到该写的响应被消费（同步模式；异步模式不设 in_flight=2）
        if (tool_idx < in_flight_type_.size() && in_flight_type_[tool_idx].load() == 2)
            return false;
        const bool pos_changed = CommandChangeDetector::hasChanged(last_gripper_command_[gi], gripper_position_command_[gi]);
        const bool trq_changed = (gi < gripper_effort_command_.size() && gi < last_gripper_effort_ack_.size() &&
            (std::isnan(last_gripper_effort_ack_[gi]) ||
             CommandChangeDetector::hasChanged(last_gripper_effort_ack_[gi], gripper_effort_command_[gi], 0.001)));
        const bool vel_changed = (gi < gripper_velocity_command_.size() && gi < last_gripper_velocity_ack_.size() &&
            (std::isnan(last_gripper_velocity_ack_[gi]) ||
             CommandChangeDetector::hasChanged(last_gripper_velocity_ack_[gi], gripper_velocity_command_[gi], 0.001)));
        if (!pos_changed && !trq_changed && !vel_changed)
            return false;
        write_cmd_out = { gripper_position_command_[gi] };
        RCLCPP_DEBUG(get_logger(), "shouldSendToolCommand: gripper write tool_idx=%zu", tool_idx);
        return true;
    }

    bool MarvinHardware::isModbusWriteAck(const unsigned char* data_buf, size_t size)
    {
        return size >= 2 && data_buf[1] == 0x10;
    }

    void MarvinHardware::applyGripperWriteAckFromInFlight(size_t tool_idx)
    {
        if (tool_idx >= in_flight_write_command_.size())
            return;
        const std::vector<double>& write_cmd = in_flight_write_command_[tool_idx];
        if (write_cmd.empty())
            return;
        size_t gi = gripperJointIndexForTool(tool_idx);
        if (toolIsHand(tool_idx))
        {
            auto* hand = dynamic_cast<marvin_ros2_control::ModbusHand*>(toolAt(tool_idx));
            if (hand)
            {
                const size_t dof = hand->getJointCount();
                for (size_t k = 0; k < gripper_joint_name_.size() && k < last_gripper_command_.size() && k < gripper_stopped_.size(); ++k)
                {
                    if (!gripperJointBelongsToTool(k, tool_idx)) continue;
                    int li = hand->mapJointNameToIndex(gripper_joint_name_[k]);
                    if (li >= 0 && static_cast<size_t>(li) < dof)
                    {
                        last_gripper_command_[k] = write_cmd[li];
                        gripper_stopped_[k] = false;
                        if (k < last_gripper_effort_ack_.size() && k < gripper_effort_command_.size())
                            last_gripper_effort_ack_[k] = gripper_effort_command_[k];
                        if (k < last_gripper_velocity_ack_.size() && k < gripper_velocity_command_.size())
                            last_gripper_velocity_ack_[k] = gripper_velocity_command_[k];
                    }
                }
                if (tool_idx < hand_stabilized_.size() && tool_idx < hand_stable_count_.size())
                {
                    hand_stabilized_[tool_idx] = false;
                    hand_stable_count_[tool_idx] = 0;
                }
            }
        }
        else if (gi < last_gripper_command_.size())
        {
            auto* mg = dynamic_cast<ModbusGripper*>(toolAt(tool_idx));
            last_gripper_command_[gi] = write_cmd[0];
            if (!mg || !mg->hasPendingConfigWrites())
            {
                if (gi < last_gripper_effort_ack_.size() && gi < gripper_effort_command_.size())
                    last_gripper_effort_ack_[gi] = gripper_effort_command_[gi];
                if (gi < last_gripper_velocity_ack_.size() && gi < gripper_velocity_command_.size())
                    last_gripper_velocity_ack_[gi] = gripper_velocity_command_[gi];
            }
            if (gi < gripper_stopped_.size())
                gripper_stopped_[gi] = false;
            if (gi < gripper_stable_count_.size())
                gripper_stable_count_[gi] = 0;
            if (gi < gripper_previous_position_.size() && gi < gripper_position_.size())
                gripper_previous_position_[gi] = gripper_position_[gi];
        }
        if (tool_idx < in_flight_type_.size())
            in_flight_type_[tool_idx].store(0);
    }

    void MarvinHardware::tryConsumeWriteAck(size_t tool_idx)
    {
        if (tool_idx >= in_flight_type_.size() || in_flight_type_[tool_idx].load() != 2)
            return;
        GetChDataFunc get_ch = toolUsesLeftChannel(tool_idx)
            ? MarvinRs485Bus::getA() : MarvinRs485Bus::getB();
        unsigned char data_buf[256] = {0};
        long received = receiveToolResponse(data_buf, sizeof(data_buf), get_ch, toolChannel(tool_idx));
        if (received <= 0)
            return;
        if (isModbusWriteAck(data_buf, static_cast<size_t>(received)))
        {
            applyGripperWriteAckFromInFlight(tool_idx);
            return;
        }
        if (received >= 2 && (data_buf[1] == 0x03 || data_buf[1] == 0x04))
            processToolResponse(data_buf, static_cast<size_t>(received), tool_idx);
        // 收到的是读响应(FC 03/04)则只更新状态，不清除 in_flight，继续等写应答
    }

    bool MarvinHardware::isToolStopped(size_t tool_idx)
    {
        if (!has_gripper_ || toolCount() == 0)
            return true;
        bool stopped = false;
        if (toolIsHand(tool_idx))
        {
            // 仅用 stabilized 判断，不再乘 at_command
            const bool stabilized = (tool_idx < hand_stabilized_.size() && hand_stabilized_[tool_idx]);
            stopped = stabilized;
        }
        else
        {
            // Gripper stopped: trust device feedback flag (target_reached).
            auto* tool = toolAt(tool_idx);
            auto* gripper = dynamic_cast<ModbusGripper*>(tool);
            stopped = (gripper && gripper->isTargetReached());
        }
        return stopped;
    }

    long MarvinHardware::receiveToolResponse(unsigned char* data_buf, size_t buf_size, GetChDataFunc get_ch_data, long channel)
    {
        long ch = channel;
        unsigned char tmp[256] = {0};
        const long size = get_ch_data(tmp, &ch);
        if (size > 0)
        {
            const size_t copy_len = std::min(static_cast<size_t>(size), buf_size);
            std::copy(tmp, tmp + copy_len, data_buf);
            return static_cast<long>(copy_len);
        }
        return 0;
    }

    // Async per-cycle pairing: recv thread polls every ~20ms, control cycle is 100ms,
    // so the recv thread gets ~5 chances per cycle to observe a reply.
    void MarvinHardware::markFrameSent(size_t tool_idx, int kind)
    {
        if (tool_idx >= kMaxTools)
            return;
        tool_tx_total_[tool_idx].fetch_add(1, std::memory_order_relaxed);
        tool_reply_kind_[tool_idx].store(kind, std::memory_order_relaxed);
        // Set pending LAST so the recv thread sees a consistent (pending, kind) pair.
        tool_reply_pending_[tool_idx].store(true, std::memory_order_release);
    }

    void MarvinHardware::markFrameAnswered(size_t tool_idx)
    {
        if (tool_idx >= kMaxTools)
            return;
        // Only count if there really was a pending frame; avoid double-counting
        // stray frames that arrive after a timeout already cleared the pending bit.
        const bool was_pending = tool_reply_pending_[tool_idx].exchange(
            false, std::memory_order_acq_rel);
        if (was_pending)
            tool_rx_total_[tool_idx].fetch_add(1, std::memory_order_relaxed);
    }

    bool MarvinHardware::checkPrevCycleTimeout(size_t tool_idx)
    {
        if (tool_idx >= kMaxTools)
            return false;
        // Non-blocking: just inspect the flag. If still pending at the start of the
        // next cycle, the recv thread never cleared it within ~5 polls -> timeout.
        const bool was_pending = tool_reply_pending_[tool_idx].exchange(
            false, std::memory_order_acq_rel);
        if (!was_pending)
            return false;

        const auto timeouts = tool_timeout_count_[tool_idx].fetch_add(
            1, std::memory_order_relaxed) + 1;
        const int kind = tool_reply_kind_[tool_idx].load(std::memory_order_relaxed);
        if (timeouts == 1)
        {
            RCLCPP_WARN(get_logger(),
                        "tool reply timeout (first): tool_idx=%zu kind=%s channel=%ld — "
                        "no reply observed by recv thread within one control cycle",
                        tool_idx, kind == 2 ? "write" : (kind == 1 ? "read" : "?"),
                        toolChannel(tool_idx));
        }
        return true;
    }

    void MarvinHardware::emitTimeoutSummary(size_t tool_idx)
    {
        if (tool_idx >= kMaxTools)
            return;
        const auto total = tool_tx_total_[tool_idx].load(std::memory_order_relaxed);
        const auto ok = tool_rx_total_[tool_idx].load(std::memory_order_relaxed);
        const auto timeouts = tool_timeout_count_[tool_idx].load(std::memory_order_relaxed);
        const auto last_reported = tool_last_reported_timeout_[tool_idx];

        // Only log when new timeouts have accumulated since the last summary.
        if (timeouts == last_reported)
            return;

        // Rate-limit: at most one summary every 5s (control loop is 100ms — was flooding WARN).
        constexpr auto kSummaryPeriod = std::chrono::seconds(5);
        const auto now = std::chrono::steady_clock::now();
        if (tool_last_summary_tp_[tool_idx].time_since_epoch().count() != 0 &&
            now - tool_last_summary_tp_[tool_idx] < kSummaryPeriod)
        {
            return;
        }

        RCLCPP_WARN(get_logger(),
                    "tool reply timeout summary: tool_idx=%zu channel=%ld — timeouts=%llu (delta=%llu), ok=%llu, total_sent=%llu, loss=%.1f%%",
                    tool_idx, toolChannel(tool_idx),
                    static_cast<unsigned long long>(timeouts),
                    static_cast<unsigned long long>(timeouts - last_reported),
                    static_cast<unsigned long long>(ok),
                    static_cast<unsigned long long>(total),
                    total > 0 ? 100.0 * static_cast<double>(timeouts) / static_cast<double>(total) : 0.0);
        tool_last_reported_timeout_[tool_idx] = timeouts;
        tool_last_summary_tp_[tool_idx] = now;
    }

    void MarvinHardware::accountSyncFrame(size_t tool_idx, int kind, bool got_reply)
    {
        if (tool_idx >= kMaxTools)
            return;
        tool_tx_total_[tool_idx].fetch_add(1, std::memory_order_relaxed);
        tool_reply_kind_[tool_idx].store(kind, std::memory_order_relaxed);
        if (got_reply)
        {
            tool_rx_total_[tool_idx].fetch_add(1, std::memory_order_relaxed);
            return;
        }
        const auto timeouts = tool_timeout_count_[tool_idx].fetch_add(1, std::memory_order_relaxed) + 1;
        if (timeouts == 1)
        {
            RCLCPP_WARN(get_logger(),
                        "tool reply timeout (first): tool_idx=%zu kind=%s channel=%ld — "
                        "sync read/write got no reply",
                        tool_idx, kind == 2 ? "write" : (kind == 1 ? "read" : "?"),
                        toolChannel(tool_idx));
        }
    }




    void MarvinHardware::processToolResponse(const unsigned char* data_buf, size_t size, size_t gripper_idx)
    {
        auto* tool = toolAt(gripper_idx);
        if (!tool) return;

        if (size >= 2 && isModbusWriteAck(data_buf, size))
            return;

        auto* gripper = dynamic_cast<ModbusGripper*>(tool);
        if (gripper)
        {
            int torque = 0, velocity = 0;
            double position = 0.0;
            if (gripper->processReadResponse(data_buf, size, torque, velocity, position))
            {
                if (gripper_idx < tool_has_valid_state_.size())
                    tool_has_valid_state_[gripper_idx].store(true);
                if (gripper_idx < tool_hb_last_rx_ms_.size())
                {
                    const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now().time_since_epoch()).count();
                    tool_hb_last_rx_ms_[gripper_idx].store(static_cast<std::int64_t>(now_ms));
                }
                if (gripper_idx < tool_hb_offline_reported_.size())
                    tool_hb_offline_reported_[gripper_idx].store(false);
                size_t gi = gripperJointIndexForTool(gripper_idx);
                updateGripperState(gi, position, velocity, torque);
                if (gripper->isTargetReached() && gi < gripper_stopped_.size())
                    gripper_stopped_[gi] = true;
            }
            return;
        }

        if (toolIsHand(gripper_idx))
        {
            auto* hand = dynamic_cast<ModbusHand*>(tool);
            if (hand)
            {
                std::vector<double> positions;
                if (hand->processReadResponse(data_buf, size, positions))
                {
                    if (gripper_idx < tool_has_valid_state_.size())
                        tool_has_valid_state_[gripper_idx].store(true);
                    if (gripper_idx < tool_hb_last_rx_ms_.size())
                    {
                        const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count();
                        tool_hb_last_rx_ms_[gripper_idx].store(static_cast<std::int64_t>(now_ms));
                    }
                    if (gripper_idx < tool_hb_offline_reported_.size())
                        tool_hb_offline_reported_[gripper_idx].store(false);
                    const size_t n = std::min(gripper_joint_name_.size(), gripper_position_.size());
                    const size_t hand_dof = hand->getJointCount();
                    const bool was_initializing =
                        (gripper_idx < tool_initial_read_done_.size() && !tool_initial_read_done_[gripper_idx]);
                    if (positions.size() == hand_dof)
                    {
                        for (size_t gi = 0; gi < n; ++gi)
                        {
                            if (!gripperJointBelongsToTool(gi, gripper_idx)) continue;
                            int local_index = hand->mapJointNameToIndex(gripper_joint_name_[gi]);
                            if (local_index >= 0 && static_cast<size_t>(local_index) < hand_dof)
                                updateGripperState(gi, positions[local_index], 0, 0);
                        }
                        if (was_initializing)
                        {
                            const size_t n_j = std::min(gripper_joint_name_.size(),
                                                        std::min(gripper_position_.size(),
                                                                 std::min(gripper_position_command_.size(), last_gripper_command_.size())));
                            std::vector<double> current;
                            current.reserve(hand_dof);
                            for (size_t gi = 0; gi < n_j; ++gi)
                            {
                                if (!gripperJointBelongsToTool(gi, gripper_idx)) continue;
                                const double pos = gripper_position_[gi];
                                gripper_position_command_[gi] = pos;
                                last_gripper_command_[gi] = pos;
                                if (gi < gripper_stopped_.size())
                                    gripper_stopped_[gi] = true;
                                current.push_back(pos);
                            }
                            if (gripper_idx < hand_previous_position_.size() &&
                                gripper_idx < hand_stable_count_.size() &&
                                gripper_idx < hand_stabilized_.size())
                            {
                                hand_previous_position_[gripper_idx] = current;
                                hand_stable_count_[gripper_idx] = kHandStableFrameCount;
                                hand_stabilized_[gripper_idx] = true;
                            }
                            syncHandDynamicsAckToCommand(gripper_idx);
                        }

                        // Steady state: current frame same as previous for N consecutive reads -> stop read polling
                        if (gripper_idx < hand_previous_position_.size() && gripper_idx < hand_stable_count_.size())
                        {
                            std::vector<double> current;
                            current.reserve(hand_dof);
                            for (size_t gi = 0; gi < n; ++gi)
                            {
                                if (!gripperJointBelongsToTool(gi, gripper_idx)) continue;
                                if (gi < gripper_position_.size())
                                    current.push_back(gripper_position_[gi]);
                            }
                            if (current.size() == hand_dof)
                            {
                                constexpr double kEpsilon = 0.001;
                                bool no_diff = (hand_previous_position_[gripper_idx].size() == current.size());
                                if (no_diff)
                                    for (size_t j = 0; j < current.size(); ++j)
                                        if (std::abs(current[j] - hand_previous_position_[gripper_idx][j]) > kEpsilon)
                                        { no_diff = false; break; }
                                if (no_diff)
                                    hand_stable_count_[gripper_idx]++;
                                else
                                    hand_stable_count_[gripper_idx] = 0;
                                hand_previous_position_[gripper_idx] = current;
                                if (hand_stable_count_[gripper_idx] >= kHandStableFrameCount)
                                {
                                    if (!hand_stabilized_[gripper_idx])
                                        RCLCPP_INFO(get_logger(), "Hand tool_idx=%zu: steady state reached (%zu consecutive frames unchanged)",
                                                    gripper_idx, hand_stable_count_[gripper_idx]);
                                    hand_stabilized_[gripper_idx] = true;
                                }
                            }
                        }
                    }
                }
            }
        }
    }


   bool MarvinHardware::connectToHardware()
    {
        RCLCPP_INFO(get_logger(), "Connecting to Marvin at %s:%d", device_ip_.c_str(), device_port_);

        // 解析 IP 地址
        std::vector<int> ip_parts;
        std::string ip_str = device_ip_;
        std::istringstream iss(ip_str);
        std::string token;
        while (std::getline(iss, token, '.'))
        {
            if (!token.empty())
            {
                ip_parts.push_back(std::stoi(token));
            }
        }
        
        if (ip_parts.size() != 4)
        {
            RCLCPP_ERROR(get_logger(), "Invalid IP address format: %s", device_ip_.c_str());
            return false;
        }
        
        // 尝试连接
        hardware_connected_ = OnLinkTo(ip_parts[0], ip_parts[1], ip_parts[2], ip_parts[3]);

        if (!hardware_connected_)
        {
            RCLCPP_ERROR(get_logger(), "Failed to connect to Marvin hardware: OnLinkTo returned false");
            return false;
        }

        // 等待连接建立并清错
        usleep(100000);
        OnClearSet();
        OnClearErr_A();
        OnClearErr_B();
        OnSetSend();
        usleep(100000);

        // 验证连接：通过检查帧序列号是否在变化来确认连接是否真的有效
        // 参考 SDK demo 中的验证方法
        DCSS test_frame_data;
        int motion_tag = 0;
        int frame_update = 0;
        
        for (int i = 0; i < 5; i++)
        {
            OnGetBuf(&test_frame_data);
            int current_frame = test_frame_data.m_Out[0].m_OutFrameSerial;
            
            if (current_frame != 0 && frame_update != current_frame)
            {
                motion_tag++;
                frame_update = current_frame;
            }
            usleep(100000);  // 等待100ms
        }

        if (motion_tag > 0)
        {
            RCLCPP_INFO(get_logger(), "Successfully connected to Marvin hardware (verified by frame updates)");

            const long sdk_major = OnGetSDKVersion();
            char version_param[30] = "VERSION";
            long ctrl_version = 0;
            if (OnGetIntPara(version_param, &ctrl_version) == 0)
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Marvin controller version: %ld (SDK major %ld, sub %ld)",
                    ctrl_version, sdk_major, ctrl_version % 100);
            }
            else
            {
                RCLCPP_WARN(
                    get_logger(),
                    "Failed to read Marvin controller VERSION parameter (SDK major %ld)",
                    sdk_major);
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to verify connection: No frame updates received from robot. Please check network connection and robot status.");
            hardware_connected_ = false;
            OnRelease();  // 释放连接
        }

        return hardware_connected_;
    }

    void MarvinHardware::disconnectFromHardware()
    {
        RCLCPP_INFO(get_logger(), "Disconnecting from Marvin hardware");

        OnRelease();

        hardware_connected_ = false;
        RCLCPP_INFO(get_logger(), "Disconnected from Marvin hardware");
    }

    bool MarvinHardware::readFromHardware(bool initial_frame)
    {
        OnGetBuf(&frame_data_);

        // Publish encoded error codes when non-zero:
        //  - Left arm:  0x600000 + m_ERRCode
        //  - Right arm: 0x610000 + m_ERRCode
        // read() returns ERROR when arm_ok is false → ResourceManager calls on_error().
        if (hardware_error_pub_)
        {
            auto publish_error_code = [&](int arm_index, int err_code)
            {
                if (err_code == 0)
                    return;

                int base = 0;
                const char* arm_name = "Unknown";
                if (arm_index == ARM_LEFT)
                {
                    base = 0x600000;
                    arm_name = "Left";
                }
                else if (arm_index == ARM_RIGHT)
                {
                    base = 0x610000;
                    arm_name = "Right";
                }
                else
                {
                    return;
                }

                const int encoded = base + err_code;

                std_msgs::msg::Int64 msg;
                msg.data = static_cast<int64_t>(encoded);
                hardware_error_pub_->publish(msg);

                RCLCPP_ERROR(get_logger(),
                             "Tianji robot error on %s arm: raw_err=%d, encoded_err=0x%X",
                             arm_name, err_code, encoded);
            };

            if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_RIGHT)
            {
                int err_code = frame_data_.m_State[robot_arm_index_].m_ERRCode;
                publish_error_code(robot_arm_index_, err_code);
            }
            else if (robot_arm_index_ == ARM_DUAL)
            {
                int left_err_code = frame_data_.m_State[ARM_LEFT].m_ERRCode;
                int right_err_code = frame_data_.m_State[ARM_RIGHT].m_ERRCode;
                publish_error_code(ARM_LEFT, left_err_code);
                publish_error_code(ARM_RIGHT, right_err_code);
            }
        }
        
        if (initial_frame)
        {
            previous_message_frame_ = frame_data_.m_Out[robot_arm_index_].m_OutFrameSerial;
        }
        const auto copyArmFeedback = [&](int arm_idx, size_t dst_offset, const char* name_prefix) {
            for (size_t i = 0; i < 7; i++)
            {
                const double pos_raw = frame_data_.m_Out[arm_idx].m_FB_Joint_Pos[i];
                const double vel_raw = frame_data_.m_Out[arm_idx].m_FB_Joint_Vel[i];
                const double effort_raw = frame_data_.m_Out[arm_idx].m_FB_Joint_SToq[i];

                const size_t dst_i = dst_offset + i;

                if (std::isnan(pos_raw) || std::isinf(pos_raw))
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_node()->get_clock(), 1000,
                        "%s %zu position is NaN/Inf, using previous value: %.3f", name_prefix, i, hw_position_states_[dst_i]);
                }
                else
                {
                    hw_position_states_[dst_i] = degreeToRad(pos_raw);
                }

                if (std::isnan(vel_raw) || std::isinf(vel_raw))
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_node()->get_clock(), 1000,
                        "%s %zu velocity is NaN/Inf, using previous value: %.3f", name_prefix, i, hw_velocity_states_[dst_i]);
                }
                else
                {
                    hw_velocity_states_[dst_i] = vel_raw;
                }

                if (std::isnan(effort_raw) || std::isinf(effort_raw))
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_node()->get_clock(), 1000,
                        "%s %zu effort is NaN/Inf, using previous value: %.3f", name_prefix, i, hw_effort_states_[dst_i]);
                }
                else
                {
                    hw_effort_states_[dst_i] = effort_raw;
                }
            }
        };

        if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_RIGHT)
        {
            copyArmFeedback(robot_arm_index_, 0, "Joint");
        }
        else if (robot_arm_index_ == ARM_DUAL)
        {
            copyArmFeedback(ARM_LEFT, 0, "Left arm joint");
            copyArmFeedback(ARM_RIGHT, 7, "Right arm joint");
        }
        previous_message_frame_ = frame_data_.m_Out[robot_arm_index_].m_OutFrameSerial;

        const bool arm_ok = (robot_arm_index_ == ARM_DUAL)
            ? (frame_data_.m_State[ARM_LEFT].m_ERRCode == 0 &&
               frame_data_.m_State[ARM_RIGHT].m_ERRCode == 0)
            : (frame_data_.m_State[robot_arm_index_].m_ERRCode == 0);
        if (previous_message_frame_ - frame_data_.m_Out[robot_arm_index_].m_OutFrameSerial < 2)
        {
            return arm_ok;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Missing more than 2 frames");
            return arm_ok;
        }
    }

    bool MarvinHardware::writeToHardware(std::vector<double>& hw_commands)
    {
        if (robot_ctrl_mode_ == "POWER_OFF") {
            return true;
        }

        bool result = true;
        OnClearSet();
        if (robot_arm_index_ == ARM_LEFT)
        {
            if (!left_brake_released_)
                result = OnSetJointCmdPos_A(hw_commands.data());
        }
        else if (robot_arm_index_ == ARM_RIGHT)
        {
            if (!right_brake_released_)
                result = OnSetJointCmdPos_B(hw_commands.data());
        }
        else if (robot_arm_index_ == ARM_DUAL)
        {
            if (!left_brake_released_)
                result = OnSetJointCmdPos_A(hw_commands.data());
            if (!right_brake_released_)
                result = OnSetJointCmdPos_B(hw_commands.data() + 7) && result;
        }
        OnSetSend();
        return result;
    }

    void MarvinHardware::loadKwr75FtConfig()
    {
        kwr75_ft_config_ = Kwr75FtConfig::defaults();
        kwr75_ft_config_.enabled = get_node_param("kwr75_ft_enabled", kwr75_ft_config_.enabled);
        kwr75_ft_config_.left_enabled = get_node_param("left_ft_enabled", kwr75_ft_config_.left_enabled);
        kwr75_ft_config_.right_enabled = get_node_param("right_ft_enabled", kwr75_ft_config_.right_enabled);
        kwr75_ft_config_.left_channel = get_node_param("left_ft_channel", kwr75_ft_config_.left_channel);
        kwr75_ft_config_.right_channel = get_node_param("right_ft_channel", kwr75_ft_config_.right_channel);
        kwr75_ft_config_.poll_interval_ms = get_node_param("ft_poll_interval_ms", kwr75_ft_config_.poll_interval_ms);
        kwr75_ft_config_.command_code = static_cast<uint8_t>(
            get_node_param("ft_command_code", static_cast<int>(kwr75_ft_config_.command_code)));
        kwr75_ft_config_.convert_to_si = get_node_param("ft_convert_to_si", kwr75_ft_config_.convert_to_si);
        kwr75_ft_config_.gravity = get_node_param("ft_gravity", kwr75_ft_config_.gravity);
        kwr75_ft_config_.warmup_timeout_ms =
            get_node_param("ft_warmup_timeout_ms", kwr75_ft_config_.warmup_timeout_ms);
        kwr75_ft_config_.left_frame_id =
            get_node_param<std::string>("left_ft_frame_id", kwr75_ft_config_.left_frame_id);
        kwr75_ft_config_.right_frame_id =
            get_node_param<std::string>("right_ft_frame_id", kwr75_ft_config_.right_frame_id);
        kwr75_ft_config_.left_wrench_topic =
            get_node_param<std::string>("left_wrench_topic", kwr75_ft_config_.left_wrench_topic);
        kwr75_ft_config_.right_wrench_topic =
            get_node_param<std::string>("right_wrench_topic", kwr75_ft_config_.right_wrench_topic);
    }

    void MarvinHardware::startKwr75FtThreads()
    {
        if (kwr75_ft_running_.exchange(true))
        {
            return;
        }

        const bool poll_left =
            kwr75_ft_config_.left_enabled &&
            (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL);
        const bool poll_right =
            kwr75_ft_config_.right_enabled &&
            (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL);

        if (poll_left)
        {
            if (kwr75_ft_thread_left_.joinable())
            {
                kwr75_ft_thread_left_.join();
            }
            kwr75_ft_thread_left_ = std::thread(&MarvinHardware::kwr75FtThread, this, ARM_LEFT);
        }
        if (poll_right)
        {
            if (kwr75_ft_thread_right_.joinable())
            {
                kwr75_ft_thread_right_.join();
            }
            kwr75_ft_thread_right_ = std::thread(&MarvinHardware::kwr75FtThread, this, ARM_RIGHT);
        }

        RCLCPP_INFO(get_logger(),
                    "KWR75 FT threads started (cmd=0x%02X, left=%s, right=%s)",
                    kwr75_ft_config_.command_code,
                    poll_left ? "on" : "off",
                    poll_right ? "on" : "off");
    }

    void MarvinHardware::stopKwr75FtThreads()
    {
        kwr75_ft_running_.store(false);
        if (kwr75_ft_thread_left_.joinable())
        {
            kwr75_ft_thread_left_.join();
        }
        if (kwr75_ft_thread_right_.joinable())
        {
            kwr75_ft_thread_right_.join();
        }
    }

    void MarvinHardware::applyKwr75Wrench(int arm_index, const std::array<double, 6>& wrench)
    {
        const bool is_left = (arm_index == ARM_LEFT);
        if (is_left)
        {
            left_ft_state_ = wrench;
        }
        else
        {
            right_ft_state_ = wrench;
        }

        auto pub = is_left ? left_wrench_pub_ : right_wrench_pub_;
        if (pub)
        {
            geometry_msgs::msg::WrenchStamped msg;
            msg.header.stamp = node_->now();
            msg.header.frame_id = is_left ? kwr75_ft_config_.left_frame_id : kwr75_ft_config_.right_frame_id;
            msg.wrench.force.x = wrench[0];
            msg.wrench.force.y = wrench[1];
            msg.wrench.force.z = wrench[2];
            msg.wrench.torque.x = wrench[3];
            msg.wrench.torque.y = wrench[4];
            msg.wrench.torque.z = wrench[5];
            pub->publish(msg);
        }
    }

    void MarvinHardware::publishKwr75SyncedToControl()
    {
        auto publish_side = [this](bool left, int arm_index) {
            const bool side_on = left ? kwr75_ft_config_.left_enabled : kwr75_ft_config_.right_enabled;
            if (!side_on)
            {
                return;
            }
            auto& slot = left ? left_kwr75_sample_ : right_kwr75_sample_;
            std::array<float, Kwr75Protocol::kAxisCount> raw {};
            if (slot.tryConsume(raw))
            {
                std::array<double, Kwr75Protocol::kAxisCount> wrench {};
                Kwr75Protocol::rawToSi(
                    raw, wrench, kwr75_ft_config_.convert_to_si, kwr75_ft_config_.gravity);
                applyKwr75Wrench(arm_index, wrench);
                return;
            }
            if (slot.has_any.load(std::memory_order_acquire))
            {
                applyKwr75Wrench(arm_index, left ? left_ft_state_ : right_ft_state_);
            }
        };

        if (robot_arm_index_ == ARM_LEFT || robot_arm_index_ == ARM_DUAL)
        {
            publish_side(true, ARM_LEFT);
        }
        if (robot_arm_index_ == ARM_RIGHT || robot_arm_index_ == ARM_DUAL)
        {
            publish_side(false, ARM_RIGHT);
        }
    }

    void MarvinHardware::kwr75FtThread(int arm_index)
    {
        Send485Func send_485 = (arm_index == ARM_LEFT) ? MarvinRs485Bus::sendA() : MarvinRs485Bus::sendB();
        const long ft_channel = (arm_index == ARM_LEFT) ? kwr75_ft_config_.left_channel
                                                        : kwr75_ft_config_.right_channel;
        const char* side_label = (arm_index == ARM_LEFT) ? "left" : "right";
        Kwr75LockFreeSample* sample_slot =
            (arm_index == ARM_LEFT) ? &left_kwr75_sample_ : &right_kwr75_sample_;

        Kwr75Marvin485Client client(
            send_485, ft_channel, sample_slot,
            kwr75_ft_config_.command_code, kwr75_ft_config_.convert_to_si,
            kwr75_ft_config_.gravity, kwr75_ft_config_.warmup_timeout_ms);

        if (client.warmup())
        {
            RCLCPP_INFO(get_logger(),
                        "KWR75 ready on %s COM2 (ch=%ld, cmd=0x%02X, %s)",
                        side_label, ft_channel, kwr75_ft_config_.command_code,
                        kwr75UsesPollThread() ? "poll thread" : "synced to control");
        }
        else
        {
            RCLCPP_WARN(get_logger(),
                        "KWR75 warmup pending on %s COM2 (ch=%ld); publishing when frames arrive",
                        side_label, ft_channel);
        }

        // Default: only start stream (0x48); publish happens in read().
        if (!kwr75UsesPollThread())
        {
            while (kwr75_ft_running_.load() && hardware_connected_)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            return;
        }

        const int interval_ms = std::max(1, kwr75_ft_config_.poll_interval_ms);
        std::array<double, Kwr75Protocol::kAxisCount> last_wrench {};
        bool have_wrench = false;

        while (kwr75_ft_running_.load() && hardware_connected_)
        {
            const auto cycle_start = std::chrono::steady_clock::now();
            const auto cycle_end = cycle_start + std::chrono::milliseconds(interval_ms);

            std::array<double, Kwr75Protocol::kAxisCount> wrench {};
            const bool got_new = client.isPollMode()
                ? client.pollAndTakeWrench(cycle_end, wrench)
                : client.takeWrench(wrench);
            if (got_new)
            {
                last_wrench = wrench;
                have_wrench = true;
            }
            if (have_wrench)
            {
                applyKwr75Wrench(arm_index, last_wrench);
            }
            std::this_thread::sleep_until(cycle_end);
        }
    }

} // namespace marvin_ros2_control


PLUGINLIB_EXPORT_CLASS(marvin_ros2_control::MarvinHardware, hardware_interface::SystemInterface)
