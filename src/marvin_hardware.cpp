#include "marvin_hardware.h"

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
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include "gripper_hardware_common/ChangingtekGripper.h"
#include "gripper_hardware_common/JodellGripper.h"
#include "marvin_ros2_control/grippers/changingtek_gripper.h"
#include "marvin_ros2_control/grippers/jd_gripper.h"

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
    static const std::vector<double> kDefaultLeftDynParam   = {2.0, 0.0, 0.0, 96.411347, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static const std::vector<double> kDefaultRightKineParam = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static const std::vector<double> kDefaultRightDynParam  = {1.8, 0.0, 0.0, 96.411347, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    // ctrl_mode 相关：字符串 <-> 内部 mode(int) 的单一转换来源
    // mode: 1=POSITION, 2=JOINT_IMPEDANCE, 3=CART_IMPEDANCE
    static int ctrlModeStringToMode(const std::string& ctrl_mode_raw, const rclcpp::Logger& logger)
    {
        const std::string normalized = normalizeString(ctrl_mode_raw);
        if (normalized == "POSITION") return 1;
        if (normalized == "JOINT_IMPEDANCE") return 2;
        if (normalized == "CART_IMPEDANCE") return 3;
        RCLCPP_WARN(logger, "Invalid ctrl_mode value: %s, defaulting to POSITION", ctrl_mode_raw.c_str());
        return 1;
    }

    static const char* modeToCtrlModeString(int mode)
    {
        switch (mode) {
            case 1: return "POSITION";
            case 2: return "JOINT_IMPEDANCE";
            case 3: return "CART_IMPEDANCE";
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
        ensure_param("gripper_type", std::string(""), hw_find("gripper_type"),
                     [](const std::string& s, const std::string& def) { (void)def; return s; });
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

        // Gripper tool parameters - declared as std::vector<double> (array of doubles)
        // kineParam: kinematic parameters (6 values), dynPara: dynamic parameters (10 values)
        ensure_double_array_sized("left_kine_param", kDefaultLeftKineParam, 6);
        ensure_double_array_sized("left_dyn_param", kDefaultLeftDynParam, 10);
        ensure_double_array_sized("right_kine_param", kDefaultRightKineParam, 6);
        ensure_double_array_sized("right_dyn_param", kDefaultRightDynParam, 10);
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

        // 先把 hardware_parameters 的初始值灌入并声明成 ROS2 参数（便于后续动态修改）
        declare_node_parameters();
        
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

        // 获取夹爪类型参数
        std::string gripper_type_raw = get_node_param("gripper_type", std::string(""));
        gripper_type_ = normalizeString(gripper_type_raw);

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
        gripper_initilized_ = false;
        gripper_joint_name_ = {};
        
        // 无论是否有夹爪，都需要调用contains_gripper()来填充joint_names_向量
        // 该函数会根据gripper_type_决定是否检测夹爪关节，并将非夹爪关节添加到joint_names_中
        contains_gripper();
        RCLCPP_INFO(get_logger(), "has_gripper_: %d", has_gripper_);
        RCLCPP_INFO(get_logger(), "gripper_type_: %s", gripper_type_.c_str());
        
        // 构建夹爪对象与其 command/state 缓存（严格保证：单臂=1个，双臂=2个；避免重复创建导致通道干扰）
        gripper_ptr_.clear();
        if (has_gripper_ && !gripper_type_.empty() && gripper_type_ != "NONE")
        {
            const size_t expected_grippers = (robot_arm_index_ == ARM_DUAL) ? 2 : 1;

            gripper_position_command_.assign(expected_grippers, -1.0);
            gripper_position_.assign(expected_grippers, 0.0);
            gripper_velocity_.assign(expected_grippers, 0.0);
            gripper_effort_.assign(expected_grippers, 0.0);
            last_gripper_command_.assign(expected_grippers, -1.0);
            last_gripper_position_.assign(expected_grippers, -1.0);
            gripper_stopped_.assign(expected_grippers, true);
            step_size_.assign(expected_grippers, 0.0);

            gripper_ptr_.reserve(expected_grippers);
            if (robot_arm_index_ == ARM_LEFT)
            {
                // Left arm gripper uses A channel
                gripper_ptr_.emplace_back(createGripper(OnClearChDataA, OnSetChDataA, OnGetChDataA));
            }
            else if (robot_arm_index_ == ARM_RIGHT)
            {
                // Right arm gripper uses B channel
                gripper_ptr_.emplace_back(createGripper(OnClearChDataB, OnSetChDataB, OnGetChDataB));
            }
            else if (robot_arm_index_ == ARM_DUAL)
            {
                // Dual arm: [0]=A(left), [1]=B(right)
                gripper_ptr_.emplace_back(createGripper(OnClearChDataA, OnSetChDataA, OnGetChDataA));
                gripper_ptr_.emplace_back(createGripper(OnClearChDataB, OnSetChDataB, OnGetChDataB));
            }

            RCLCPP_INFO(get_logger(),
                        "Gripper init: arm_type=%s, gripper_type=%s, detected_gripper_joints=%zu, created_grippers=%zu",
                        robot_arm_config_.c_str(), gripper_type_.c_str(), gripper_joint_name_.size(), gripper_ptr_.size());
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
        RCLCPP_INFO(get_logger(), "Gripper type: %s", gripper_type_.empty() ? "none" : gripper_type_.c_str());
        
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
        RCLCPP_INFO(get_logger(), "Robot configuration parameters ready. Use 'ros2 param set' to change configuration dynamically.");
            
        RCLCPP_INFO(get_logger(), "Marvin Hardware Interface initialized successfully");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

rcl_interfaces::msg::SetParametersResult
MarvinHardware::paramCallback(const std::vector<rclcpp::Parameter> & params)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    if (!hardware_connected_) {
        result.successful = false;
        result.reason = "Hardware not connected";
        return result;
    }

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

    for (const auto & param : params) {
        if (param.get_name() == "ctrl_mode") {
            std::string ctrl_mode_str = param.as_string();
            const int mode = ctrlModeStringToMode(ctrl_mode_str, get_logger());
            ctrl_mode = modeToCtrlModeString(mode);
            robot_ctrl_mode_ = ctrl_mode;
            need_config_update = true;
            ctrl_mode_changed = true;
            RCLCPP_INFO(get_logger(), "ctrl_mode parameter changed to: %s", ctrl_mode.c_str());
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

    // Apply configuration if needed
    if (need_config_update) {
        const int mode = ctrl_mode.empty() ? -1 : ctrlModeStringToMode(ctrl_mode, get_logger());
        applyRobotConfiguration(mode, drag_mode, cart_type,
                              max_joint_speed, max_joint_acceleration,
                              joint_k_gains, joint_d_gains, cart_k_gains, cart_d_gains);
        
        // If ctrl_mode changed, update the arm control mode
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
        if (is_left)
        {
            RCLCPP_INFO(get_logger(), "set tool load parameters");
        }
        OnSetSend();
        usleep(100000);

        if (robot_ctrl_mode_ == "POSITION")
        {
            OnClearSet();
            set_target_state(1); // 1:position mode
            OnSetSend();
            usleep(100000);
        }
        else
        {
            OnClearSet();
            set_target_state(3); // 3:torque mode
            set_tool(kine.data(), dyn.data());

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

    std::unique_ptr<ModbusGripper> MarvinHardware::createGripper(Clear485Func clear_485, Send485Func send_485,
                                                                 GetChDataFunc get_ch_data)
    {
        /// 启动的时候调用
        clear_485();
        // gripper_type_ has been normalized to UPPERCASE by normalizeString()
        enum class GripperKind { JD, Changingtek90C, Changingtek90D };

        static const std::unordered_map<std::string, GripperKind> kGripperTypeMap = {
            // JD / RG75
            {"RG75", GripperKind::JD},
            {"JDGRIPPER", GripperKind::JD},

            // Changingtek 90C variants (AG2F90_C is the common实际入参)
            {"CHANGINGTEK90C", GripperKind::Changingtek90C},
            {"AG2F90", GripperKind::Changingtek90C},
            {"AG2F90C", GripperKind::Changingtek90C},
            {"AG2F90_C", GripperKind::Changingtek90C},

            // Changingtek 90D variants
            {"CHANGINGTEK90D", GripperKind::Changingtek90D},
            {"AG2F90D", GripperKind::Changingtek90D},
            {"AG2F90_D", GripperKind::Changingtek90D},
        };

        const auto it = kGripperTypeMap.find(gripper_type_);
        const GripperKind kind = (it == kGripperTypeMap.end()) ? GripperKind::JD : it->second;

        switch (kind)
        {
            case GripperKind::JD:
                if (it == kGripperTypeMap.end())
                {
                    RCLCPP_WARN(get_logger(),
                                "Unknown gripper type '%s' (normalized). Using default JDGripper.",
                                gripper_type_.c_str());
                }
                RCLCPP_INFO(get_logger(), "Creating JD Gripper");
                return std::make_unique<JDGripper>(clear_485, send_485, get_ch_data);

            case GripperKind::Changingtek90C:
                RCLCPP_INFO(get_logger(), "Creating CHANGINGTEK90C Gripper");
                return std::make_unique<ChangingtekGripper90C>(clear_485, send_485, get_ch_data);

            case GripperKind::Changingtek90D:
                RCLCPP_INFO(get_logger(), "Creating CHANGINGTEK90D Gripper");
                return std::make_unique<ChangingtekGripper90D>(clear_485, send_485, get_ch_data);
        }

        // Defensive fallback
        RCLCPP_INFO(get_logger(), "Creating JD Gripper");
        return std::make_unique<JDGripper>(clear_485, send_485, get_ch_data);
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

    void MarvinHardware::contains_gripper()
    {
        int joint_index = 0;
        bool should_detect_gripper = !gripper_type_.empty() && gripper_type_ != "NONE";

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
                    RCLCPP_INFO(get_logger(), "Detected gripper joint: %s (index %zu)",
                            joint.name.c_str(), gripper_joint_index_);
                } 
            }
            
            // 如果不是夹爪关节，则添加到机械臂关节列表
            if (!is_gripper_joint)
            {
                joint_names_.push_back(joint.name);              
            }
            joint_index++;
        }
        RCLCPP_INFO(get_logger(), "Detected gripper joints: %zu, arm joints: %zu", 
                    gripper_joint_name_.size(), joint_names_.size());
    }

    hardware_interface::CallbackReturn MarvinHardware::on_activate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Activating Marvin Hardware Interface...");
        // Connect to real hardware
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
        
        // Connect and initialize gripper (this will update kineParam and dynPara if gripper detected)
        // 只有在配置了夹爪类型时才尝试连接夹爪
        if (has_gripper_ && !gripper_type_.empty() && gripper_type_ != "NONE" && !gripper_ptr_.empty())
        {
            bool gripper_connected = connect_gripper();
            if (!gripper_connected)
            {
                RCLCPP_WARN(get_logger(), "Gripper initialization failed, using default (zero) parameters");
            }
        }
        else if (!gripper_type_.empty() && gripper_type_ != "NONE")
        {
            // 配置了夹爪类型但没有检测到夹爪关节或创建夹爪对象失败
            RCLCPP_WARN(get_logger(), "Gripper type configured but no gripper detected or created");
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

        OnGetBuf(&frame_data_);
        RCLCPP_INFO(get_logger(), "current state of A arm:%d\n",
                    frame_data_.m_State[0].m_CurState);
        RCLCPP_INFO(get_logger(), "cmd state of A arm:%d\n", frame_data_.m_State[0].m_CmdState);
        RCLCPP_INFO(get_logger(), "error code of A arms:%d\n",
                    frame_data_.m_State[0].m_ERRCode);
        RCLCPP_INFO(get_logger(), "cmd of vel and acc:%d %d\n",
                    frame_data_.m_In[0].m_Joint_Vel_Ratio, frame_data_.m_In[0].m_Joint_Acc_Ratio);

        // Start gripper control threads if gripper is configured and connected
        // 只有在配置了夹爪类型且成功创建了夹爪对象时才启动控制线程
        if (has_gripper_ && !gripper_type_.empty() && gripper_type_ != "NONE" && !gripper_ptr_.empty())
        {
            RCLCPP_INFO(get_logger(), "Gripper Connected");
            // start gripper control thread
            gripper_ctrl_thread_ = std::thread(&MarvinHardware::gripper_callback, this);
            gripper_ctrl_thread_.detach();
            std::thread recv_thread(&MarvinHardware::recv_thread_func, this);
            recv_thread.detach();
            // read once on_activate
            // int cur_pos_status = 0;
            // int cur_speed_status = 0;
            // int cur_effort_status = 0;
            // bool success = ModbusGripper::getStatus(cur_effort_status, cur_speed_status, cur_pos_status, OnClearChDataA);
            // gripper_position_ = (9000 - cur_pos_status) / 9000.0;
            // gripper_position_ = 0.0;
        }
        else
        {
            // 没有配置夹爪是正常情况，不应该返回错误
            if (!gripper_type_.empty() && gripper_type_ != "NONE")
            {
                // 配置了夹爪类型但连接失败
                RCLCPP_ERROR(get_logger(), "Gripper type configured but connection failed");
                return hardware_interface::CallbackReturn::ERROR;
            }
            else
            {
                // 没有配置夹爪，这是正常情况
                RCLCPP_INFO(get_logger(), "No gripper configured, continuing without gripper");
            }
        }
        RCLCPP_INFO(get_logger(), "Marvin Hardware Interface activated successfully");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    void MarvinHardware::gripper_callback()
    {
        // Read frequency controller: read every 4 cycles to reduce Modbus load
        ReadFrequencyController read_controller(4);
        
        while (hardware_connected_)
        {
            // Read gripper status from hardware periodically
            if (read_controller.shouldRead())
            {
                // Request status updates for all grippers (sends read requests)
                // The actual status will be updated by recv_thread_func when responses arrive
                for(size_t i = 0; i < gripper_ptr_.size() && i < gripper_stopped_.size(); i++)
                {
                    if(!gripper_stopped_[i])
                    {
                        // Send read request - actual status will be updated by recv_thread_func
                        gripper_hardware_common::GripperBase* gripper_base = gripper_ptr_[i].get();
                        gripper_base->getStatus();
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
            }
            // Write commands when gripper command is different
            for(size_t i = 0; i < gripper_ptr_.size() && i < gripper_stopped_.size() && i < last_gripper_command_.size(); i++)
            {
                if (CommandChangeDetector::hasChanged(last_gripper_command_[i], gripper_position_command_[i]))
                {
                    RCLCPP_INFO(get_logger(), "Gripper %zu: writing position %.3f", i, gripper_position_command_[i]);
                    
                    // Use base class interface - move_gripper now accepts normalized position
                    gripper_hardware_common::GripperBase* gripper_base = gripper_ptr_[i].get();
                    
                    int cur_speed_set = 100;
                    int cur_effort_set = 100;
                    
                    // Use base class interface to send command with normalized position
                    // Each gripper implementation will handle the conversion internally
                    bool success = gripper_base->move_gripper(cur_effort_set, cur_speed_set, gripper_position_command_[i]);
                    if (success)
                    {
                        last_gripper_command_[i] = gripper_position_command_[i];
                        gripper_stopped_[i] = false;
                        if (i < gripper_position_.size())
                        {
                            step_size_[i] = (last_gripper_command_[i] - gripper_position_[i]) / 10.0;
                        }
                    }
                    else
                    {
                        RCLCPP_WARN(get_logger(), "Failed to send command to gripper %zu", i);
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    bool MarvinHardware::connect_gripper()
    {
        bool result = true;
        for (size_t i = 0; i < gripper_ptr_.size(); i++)
        {
            result = result && gripper_ptr_[i]->initialize();
        }
        return result;
    }

    hardware_interface::CallbackReturn MarvinHardware::on_deactivate(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Deactivating Marvin Hardware Interface...");
        OnClearSet();
        if (robot_arm_index_ == ARM_LEFT)
        {
            OnSetDragSpace_A(0);
            OnSetTargetState_A(0);
        }
        else if (robot_arm_index_ == ARM_RIGHT)
        {
            OnSetTargetState_B(0);
        }
        else if (robot_arm_index_ == ARM_DUAL)
        {
            OnSetTargetState_A(0);
            OnSetTargetState_B(0);
        }

        OnSetSend();
        usleep(100000);

        if (hardware_connected_)
        {
            disconnectFromHardware();
        }
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MarvinHardware::on_shutdown(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_INFO(get_logger(), "Shutting down Marvin Hardware Interface...");

        if (hardware_connected_)
        {
            disconnectFromHardware();
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn MarvinHardware::on_error(
        const rclcpp_lifecycle::State& /*previous_state*/)
    {
        RCLCPP_ERROR(get_logger(), "Error in Marvin Hardware Interface");

        // Attempt to safely disconnect from hardware
        if (hardware_connected_)
        {
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

                state_interfaces.push_back(
                    std::make_shared<hardware_interface::StateInterface>(
                        gripper_joint_name_[i],
                        hardware_interface::HW_IF_VELOCITY,
                        &gripper_velocity_[i]));

                state_interfaces.push_back(
                    std::make_shared<hardware_interface::StateInterface>(
                        gripper_joint_name_[i],
                        hardware_interface::HW_IF_EFFORT,
                        &gripper_effort_[i]));
            }
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

    void MarvinHardware::updateGripperState(size_t gripper_idx, double position, int velocity, int torque)
    {
        if (gripper_idx >= gripper_position_.size() || gripper_idx >= gripper_velocity_.size() || 
            gripper_idx >= gripper_effort_.size())
        {
            return;
        }

        gripper_position_[gripper_idx] = position;
        gripper_velocity_[gripper_idx] = static_cast<double>(velocity);
        gripper_effort_[gripper_idx] = static_cast<double>(torque);

        // Update gripper_stopped_ based on position change
        if (gripper_idx < last_gripper_position_.size() && gripper_idx < gripper_stopped_.size())
        {
            if (last_gripper_position_[gripper_idx] < 0.0)
            {
                last_gripper_position_[gripper_idx] = position;
                gripper_stopped_[gripper_idx] = false;
            }
            else if (CommandChangeDetector::hasChanged(position, last_gripper_position_[gripper_idx]))
            {
                last_gripper_position_[gripper_idx] = position;
                gripper_stopped_[gripper_idx] = false;
            }
            else
            {
                gripper_stopped_[gripper_idx] = true;
            }
        }
    }

    bool MarvinHardware::recv_thread_func()
    {
        struct ChannelSpec
        {
            const char* name;
            GetChDataFunc get_ch_data;
            size_t gripper_idx;
        };

        long ch = 2;
        unsigned char data_buf[256] = {0};

        while (hardware_connected_)
        {
            if (!has_gripper_ || gripper_ptr_.empty())
            {
                usleep(10 * 1000);
                continue;
            }

            // Determine which channels to poll based on arm configuration.
            // Mapping rule:
            // - LEFT:  idx0 <- channel A
            // - RIGHT: idx0 <- channel B
            // - DUAL:  idx0 <- channel A, idx1 <- channel B
            ChannelSpec specs[2];
            size_t spec_count = 0;
            if (robot_arm_index_ == ARM_LEFT)
            {
                specs[0] = {"A", OnGetChDataA, 0};
                spec_count = 1;
            }
            else if (robot_arm_index_ == ARM_RIGHT)
            {
                specs[0] = {"B", OnGetChDataB, 0};
                spec_count = 1;
            }
            else if (robot_arm_index_ == ARM_DUAL)
            {
                specs[0] = {"A", OnGetChDataA, 0};
                specs[1] = {"B", OnGetChDataB, 1};
                spec_count = 2;
                if (gripper_ptr_.size() < 2)
                {
                    RCLCPP_WARN_THROTTLE(get_logger(), *get_node()->get_clock(), 2000,
                        "Dual-arm gripper configured but created_grippers=%zu (<2). Right gripper feedback may be missing.",
                        gripper_ptr_.size());
                }
            }

            for (size_t si = 0; si < spec_count; ++si)
            {
                const auto& spec = specs[si];
                if (spec.gripper_idx >= gripper_ptr_.size())
                {
                    continue;
                }

                const long size = spec.get_ch_data(data_buf, &ch);
                if (size <= 0)
                {
                    continue;
                }

                int torque = 0, velocity = 0;
                double position = 0.0;
                if (gripper_ptr_[spec.gripper_idx]->processReadResponse(
                        data_buf, static_cast<size_t>(size), torque, velocity, position))
                {
                    updateGripperState(spec.gripper_idx, position, velocity, torque);
                }
            }

            usleep(10 * 1000);
        }
        return true;
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

        if (previous_message_frame_ - frame_data_.m_Out[robot_arm_index_].m_OutFrameSerial < 2)
        {
            return true;
        }
        else
        {
            RCLCPP_INFO(get_logger(), "Missing more than 2 frames");
            return true;
        }
    }

    bool MarvinHardware::writeToHardware(std::vector<double>& hw_commands)
    {
        bool result = true;
        OnClearSet();
        if (robot_arm_index_ == ARM_LEFT)
        {
            result = OnSetJointCmdPos_A(hw_commands.data());
        }
        else if (robot_arm_index_ == ARM_RIGHT)
        {
            result = OnSetJointCmdPos_B(hw_commands.data());
        }
        else if (robot_arm_index_ == ARM_DUAL)
        {
            result = OnSetJointCmdPos_A(hw_commands.data());
            result = result && OnSetJointCmdPos_B(hw_commands.data() + 7);
        }
        OnSetSend();
        return result;
    }

} // namespace marvin_ros2_control


PLUGINLIB_EXPORT_CLASS(marvin_ros2_control::MarvinHardware, hardware_interface::SystemInterface)
