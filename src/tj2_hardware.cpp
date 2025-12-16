#include "tj2_hardware.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <random>
#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <sstream>
#include <algorithm>

namespace marvin_ros2_control
{
 

MarvinHardware::~MarvinHardware()
{
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Destroying TJ2 Hardware Interface...");
  
  // Ensure hardware is disconnected
  if (hardware_connected_) {
    disconnectFromHardware();
  }
  
  RCLCPP_DEBUG(rclcpp::get_logger("MarvinHardware"), "TJ2 Hardware Interface destroyed");
}

hardware_interface::CallbackReturn MarvinHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Initializing TJ2 Hardware Interface...");

  logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger("MarvinHardware"));
  clock_ = std::make_shared<rclcpp::Clock>();
  node_ = get_node();
  
  // Robot operation mode: 1=Position, 2=Joint Impedance, 3=Cartesian Impedance
  node_->declare_parameter<int>("mode", 2);
  // Arm side: 0=left, 1=right, 2=dual (both)
  node_->declare_parameter<int>("arm_side", 2);
  // Drag mode: -1=don't update, 0=disable, 1=joint space drag, 2=cartesian space drag
  node_->declare_parameter<int>("drag_mode", -1);
  // Joint impedance K gains (7 values)
  node_->declare_parameter<std::vector<double>>("joint_k_gains", std::vector<double>{2.0, 2.0, 2.0, 1.6, 1.0, 1.0, 1.0});
  // Joint impedance D gains (7 values)
  node_->declare_parameter<std::vector<double>>("joint_d_gains", std::vector<double>{0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4});
  // Cartesian impedance K gains (7 values)
  node_->declare_parameter<std::vector<double>>("cart_k_gains", std::vector<double>{1800.0, 1800.0, 1800.0, 40.0, 40.0, 40.0, 20.0});
  // Cartesian impedance D gains (7 values)
  node_->declare_parameter<std::vector<double>>("cart_d_gains", std::vector<double>{0.6, 0.6, 0.6, 0.4, 0.4, 0.4, 0.4});
  // Cartesian impedance type
  node_->declare_parameter<int>("cart_type", 2);
  // Joint speed and acceleration limits
  node_->declare_parameter<double>("max_joint_speed", 10.0);
  node_->declare_parameter<double>("max_joint_acceleration", 10.0);
  
  // Legacy parameters (for backward compatibility)
  node_->declare_parameter<double>("max_velocity", 10.0);
  node_->declare_parameter<double>("max_acceleration", 10.0);
  node_->declare_parameter<bool>("use_drag_mode", false);
  node_->declare_parameter<int>("ctrl_mode", static_cast<int>(RobotCtrlMode::POSITION));
  node_->declare_parameter<std::vector<double>>("joint_imp_gain", std::vector<double>{2,2,2,1.6,1,1,1});
  node_->declare_parameter<std::vector<double>>("joint_imp_damp", std::vector<double>{0.4,0.4,0.4,0.4,0.4,0.4,0.4});
  node_->declare_parameter<std::vector<double>>("cart_imp_gain", std::vector<double>{500,500,500,10,10,10,0});
  node_->declare_parameter<std::vector<double>>("cart_imp_damp", std::vector<double>{0.1,0.1,0.1,0.3,0.3,1});

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
    
    // Get the number of joints from the hardware info
    size_t num_joints = info.joints.size();
    robot_arm_left_right_ = static_cast<int>(RobotArmConfig::LEFT_ARM);
    robot_ctrl_mode_ = static_cast<int>(RobotCtrlMode::POSITION);

    if (info_.hardware_parameters.find("left_right_arm") != info_.hardware_parameters.end()) {
      robot_arm_left_right_ = std::stoi(info_.hardware_parameters.at("left_right_arm"));
      RCLCPP_INFO(get_logger(), "Found left_right_arm parameter: %d", robot_arm_left_right_);
    } else {
      RCLCPP_WARN(get_logger(), "No left_right_arm parameter found, using default: %d", robot_arm_left_right_);
    }

    if (info_.hardware_parameters.find("ctrl_mode") != info_.hardware_parameters.end()) {
      robot_ctrl_mode_ = std::stoi(info_.hardware_parameters.at("ctrl_mode"));
      RCLCPP_INFO(get_logger(), "Found ctrl mode parameter: %d", robot_ctrl_mode_);
    } else {
      RCLCPP_WARN(get_logger(), "No ctrl mode found, using default: %d", robot_ctrl_mode_);
    }
   
    RCLCPP_INFO(get_logger(), "Initializing %zu joints", num_joints);

    // Resize all arrays based on the number of joints
    hw_position_commands_.resize(num_joints, 0.0);
    hw_velocity_commands_.resize(num_joints, 0.0);
    hw_position_states_.resize(num_joints, 0.0);
    hw_velocity_states_.resize(num_joints, 0.0);
    hw_effort_states_.resize(num_joints, 0.0);

    // 初始化夹爪参数 如果有的话
    has_gripper_ = false;
    gripper_joint_index_ = -1;
    gripper_initilized_ = false;
    gripper_joint_name_ = {};
    contains_gripper();

    if (has_gripper_)
    {
      if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::LEFT_ARM) || robot_arm_left_right_ == static_cast<int>(RobotArmConfig::RIGHT_ARM))
      {
        
        gripper_position_command_ = {-1.0};
        gripper_position_ = {0.0};
        gripper_velocity_ = {0.0};
        gripper_effort_ = {0.0};
        last_gripper_command_ = {-1.0};
        last_gripper_position_ = {-1.0};
        gripper_stopped_ = {true};
        step_size_ = {0.0};
        gripper_ptr_.reserve(1);
        // std::unique_ptr<marvin_ros2_control::ModbusGripper> gripper_ptr = std::make_unique<marvin_ros2_control::JDGripper>();
        if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::LEFT_ARM))
        {
          gripper_ptr_.emplace_back(std::make_unique<marvin_ros2_control::JDGripper>(OnClearChDataA, OnSetChDataA));
        }
        else
        {
          gripper_ptr_.emplace_back(std::make_unique<marvin_ros2_control::JDGripper>(OnClearChDataB, OnSetChDataB));
        }
      }
      else if(robot_arm_left_right_ == static_cast<int>(RobotArmConfig::DUAL_ARM))
      {
        
        gripper_position_command_ = {-1.0, -1.0};
        gripper_position_ = {0.0, 0.0};
        gripper_velocity_ = {0.0, 0.0};
        gripper_effort_ = {0.0, 0.0};
        last_gripper_command_ = {-1.0, -1.0};
        last_gripper_position_ = {-1.0, -1.0};
        gripper_stopped_ = {true, true};
        step_size_ = {0.0, 0.0};
        //std::unique_ptr<marvin_ros2_control::ModbusGripper> left_gripper = std::make_unique<marvin_ros2_control::JDGripper>();
        //std::unique_ptr<marvin_ros2_control::ModbusGripper> right_gripper = std::make_unique<marvin_ros2_control::JDGripper>();
        //gripper_ptr_.push_back(std::move(left_gripper));
        //gripper_ptr_.push_back(std::move(right_gripper));
        gripper_ptr_.reserve(2);
        gripper_ptr_.emplace_back(std::make_unique<marvin_ros2_control::JDGripper>(OnClearChDataA, OnSetChDataA));
        gripper_ptr_.emplace_back(std::make_unique<marvin_ros2_control::JDGripper>(OnClearChDataB, OnSetChDataB));
      }
    } 
    
    // Initialize hardware connection status
    hardware_connected_ = false;
    simulation_active_ = false;
  param_callback_handle_ = node_->add_on_set_parameters_callback(
          std::bind(&MarvinHardware::paramCallback, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "Robot configuration parameters ready. Use 'ros2 param set' to change configuration dynamically.");
    
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
    int mode = -1;
    int arm_side = -1;
    int drag_mode = -1;
    int cart_type = -1;
    double max_joint_speed = -1.0;
    double max_joint_acceleration = -1.0;
    std::vector<double> joint_k_gains;
    std::vector<double> joint_d_gains;
    std::vector<double> cart_k_gains;
    std::vector<double> cart_d_gains;

    for (const auto & param : params) {
        if (param.get_name() == "mode") {
            mode = param.as_int();
            if (mode <= 0 || mode > 3) {
                result.successful = false;
                result.reason = "Invalid mode. Must be 1 (Position), 2 (Joint Impedance), or 3 (Cartesian Impedance)";
                return result;
            }
            need_config_update = true;
            RCLCPP_INFO(get_logger(), "Mode parameter changed to: %d", mode);
        }
        else if (param.get_name() == "arm_side") {
            arm_side = param.as_int();
            if (arm_side < 0 || arm_side > 2) {
                result.successful = false;
                result.reason = "Invalid arm_side. Must be 0 (left), 1 (right), or 2 (both)";
                return result;
            }
            need_config_update = true;
            RCLCPP_INFO(get_logger(), "Arm side parameter changed to: %d", arm_side);
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
        // Legacy parameter handling
        else if (param.get_name() == "ctrl_mode") {
            int legacy_mode = param.as_int();
            // Map legacy ctrl_mode to new mode: 2=position(1), 3=joint_imp(2), 4=cart_imp(3)
            if (legacy_mode == 2) mode = 1;
            else if (legacy_mode == 3) mode = 2;
            else if (legacy_mode == 4) mode = 3;
            need_config_update = true;
            RCLCPP_WARN(get_logger(), "Using legacy ctrl_mode parameter. Consider using 'mode' instead.");
        }
    }

    // Apply configuration if needed
    if (need_config_update) {
        applyRobotConfiguration(mode, arm_side, drag_mode, cart_type, 
                              max_joint_speed, max_joint_acceleration,
                              joint_k_gains, joint_d_gains, cart_k_gains, cart_d_gains);
    }

    return result;
}

void MarvinHardware::applyRobotConfiguration(int mode, int arm_side, int drag_mode, int cart_type,
                                              double max_joint_speed, double max_joint_acceleration,
                                              const std::vector<double>& joint_k_gains,
                                              const std::vector<double>& joint_d_gains,
                                              const std::vector<double>& cart_k_gains,
                                              const std::vector<double>& cart_d_gains)
{
    // Get current parameter values if not provided
    if (mode == -1) {
        mode = node_->get_parameter("mode").as_int();
    }
    if (arm_side == -1) {
        arm_side = node_->get_parameter("arm_side").as_int();
    }
    if (drag_mode == -1) {
        drag_mode = node_->get_parameter("drag_mode").as_int();
    }
    if (cart_type == -1) {
        cart_type = node_->get_parameter("cart_type").as_int();
    }
    if (max_joint_speed <= 0.0) {
        max_joint_speed = node_->get_parameter("max_joint_speed").as_double();
    }
    if (max_joint_acceleration <= 0.0) {
        max_joint_acceleration = node_->get_parameter("max_joint_acceleration").as_double();
    }

    // Default KD values
    std::vector<double> default_joint_k = {2.0, 2.0, 2.0, 1.6, 1.0, 1.0, 1.0};
    std::vector<double> default_joint_d = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
    std::vector<double> default_cart_k = {1800.0, 1800.0, 1800.0, 40.0, 40.0, 40.0, 20.0};
    std::vector<double> default_cart_d = {0.6, 0.6, 0.6, 0.4, 0.4, 0.4, 0.4};

    // Use provided KD or get from parameters or defaults
    std::vector<double> final_joint_k = (!joint_k_gains.empty()) ? joint_k_gains :
        (node_->has_parameter("joint_k_gains") && node_->get_parameter("joint_k_gains").as_double_array().size() == 7) ?
        node_->get_parameter("joint_k_gains").as_double_array() : default_joint_k;
    std::vector<double> final_joint_d = (!joint_d_gains.empty()) ? joint_d_gains :
        (node_->has_parameter("joint_d_gains") && node_->get_parameter("joint_d_gains").as_double_array().size() == 7) ?
        node_->get_parameter("joint_d_gains").as_double_array() : default_joint_d;
    std::vector<double> final_cart_k = (!cart_k_gains.empty()) ? cart_k_gains :
        (node_->has_parameter("cart_k_gains") && node_->get_parameter("cart_k_gains").as_double_array().size() == 7) ?
        node_->get_parameter("cart_k_gains").as_double_array() : default_cart_k;
    std::vector<double> final_cart_d = (!cart_d_gains.empty()) ? cart_d_gains :
        (node_->has_parameter("cart_d_gains") && node_->get_parameter("cart_d_gains").as_double_array().size() == 7) ?
        node_->get_parameter("cart_d_gains").as_double_array() : default_cart_d;

    // Determine which arms to update
    bool update_left = (arm_side == 0 || arm_side == 2);
    bool update_right = (arm_side == 1 || arm_side == 2);

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
        OnSetSend();
        usleep(100000);
        
        robot_ctrl_mode_ = static_cast<int>(RobotCtrlMode::POSITION);
        
        // Set joint speed and acceleration limits
        OnClearSet();
        if (update_left) {
            OnSetJointLmt_A(static_cast<int>(max_joint_speed), static_cast<int>(max_joint_acceleration));
        }
        if (update_right) {
            OnSetJointLmt_B(static_cast<int>(max_joint_speed), static_cast<int>(max_joint_acceleration));
        }
        OnSetSend();
        usleep(100000);
        
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
        
        // Update drag mode if specified
        if (drag_mode >= 0) {
            if (update_left) {
                OnSetDragSpace_A(drag_mode);
            }
            if (update_right) {
                OnSetDragSpace_B(drag_mode);
            }
        }
        
        OnSetSend();
        usleep(100000);
        
        // Set joint speed and acceleration limits
        OnClearSet();
        if (update_left) {
            OnSetJointLmt_A(static_cast<int>(max_joint_speed), static_cast<int>(max_joint_acceleration));
        }
        if (update_right) {
            OnSetJointLmt_B(static_cast<int>(max_joint_speed), static_cast<int>(max_joint_acceleration));
        }
        OnSetSend();
        usleep(100000);
        
        robot_ctrl_mode_ = static_cast<int>(RobotCtrlMode::JOINT_IMPEDANCE);
        
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
        
        // Update drag mode if specified
        if (drag_mode >= 0) {
            if (update_left) {
                OnSetDragSpace_A(drag_mode);
            }
            if (update_right) {
                OnSetDragSpace_B(drag_mode);
            }
        }
        
        OnSetSend();
        usleep(100000);
        
        // Set joint speed and acceleration limits
        OnClearSet();
        if (update_left) {
            OnSetJointLmt_A(static_cast<int>(max_joint_speed), static_cast<int>(max_joint_acceleration));
        }
        if (update_right) {
            OnSetJointLmt_B(static_cast<int>(max_joint_speed), static_cast<int>(max_joint_acceleration));
        }
        OnSetSend();
        usleep(100000);
        
        robot_ctrl_mode_ = static_cast<int>(RobotCtrlMode::CART_IMPEDANCE);
        
        RCLCPP_INFO(get_logger(), "Set to cartesian impedance mode with KD=[%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f], speed=%.1f, acceleration=%.1f",
                    K[0], K[1], K[2], K[3], K[4], K[5], K[6], max_joint_speed, max_joint_acceleration);
    }
}


void MarvinHardware::setLeftArmCtrl()
{
  /// clear error first
  double kineParam[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double dynPara[10] = {2.0, 0.0, 0.0, 96.411347, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  OnClearSet();
  OnClearErr_A();
  RCLCPP_INFO(get_logger(), "set tool load parameters");
  OnSetSend();
  usleep(100000);

  if (robot_ctrl_mode_ == static_cast<int>(RobotCtrlMode::POSITION))
  {
    OnClearSet();
    OnSetTargetState_A(1) ; //3:torque mode; 1:position mode
    OnSetSend();
    usleep(100000);
  }
  else
  {
    OnClearSet();
    OnSetTargetState_A(3) ; //3:torque mode; 1:position mode
    OnSetTool_A(kineParam, dynPara);
    if (robot_ctrl_mode_ == static_cast<int>(RobotCtrlMode::JOINT_IMPEDANCE))
    {
      double K[7] = {2,2,2,1.6,1,1,1};//预设为参数最大上限，供参考。
      double D[7] = {0.4,0.4,0.4,0.4,0.4,0.4,0.4};//预设为参数最大上限，供参考。
      OnSetJointKD_A(K, D);
      OnSetImpType_A(1);
    }
    else if (robot_ctrl_mode_ == static_cast<int>(RobotCtrlMode::CART_IMPEDANCE))
    {
      double K[7] = {1800,1800,1800,40,40,40,20}; //预设为参数最大上限，供参考。
      double D[7] = {0.6,0.6,0.6,0.4,0.4,0.4, 0.4};//预设为参数最大上限，供参考。
      OnSetCartKD_A(K, D, 2);
      OnSetImpType_A(2);
    }
    OnSetSend();
    usleep(100000);
  }
  
  /// set maximum speed and acceleration
  OnClearSet();
  OnSetJointLmt_A(40, 10) ;
  OnSetSend();
  usleep(100000);
}

void MarvinHardware::setRightArmCtrl()
{
  /// clear error first
  double kineParam[6] = {0, 0, 0, 0, 0, 0};
  double dynPara[10] = {1.8, 0.0, 0.0, 96.411347, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  OnClearSet();
  OnClearErr_B();


  OnSetSend();
  usleep(100000);
  if (robot_ctrl_mode_ == static_cast<int>(RobotCtrlMode::POSITION))
  {
    OnClearSet();
    OnSetTargetState_B(1) ; //3:torque mode; 1:position mode
    OnSetSend();
    usleep(100000);
  }
  else
  {
    OnClearSet();
    OnSetTargetState_B(3) ; //3:torque mode; 1:position mode
    OnSetTool_B(kineParam, dynPara);
    if (robot_ctrl_mode_ == static_cast<int>(RobotCtrlMode::JOINT_IMPEDANCE))
    {
      double K[7] = {2,2,2,1.6,1,1,1};//预设为参数最大上限，供参考。
      double D[7] = {0.4,0.4,0.4,0.4,0.4,0.4,0.4};//预设为参数最大上限，供参考。
      OnSetJointKD_B(K, D);
      OnSetImpType_B(1);
    }
    else if (robot_ctrl_mode_ == static_cast<int>(RobotCtrlMode::CART_IMPEDANCE))
    {
      double K[7] = {2500,2500,2500,60,60,60,20}; //预设为参数最大上限，供参考。
      double D[7] = {0.6,0.6,0.6,0.2,0.2,0.2, 0.4};//预设为参数最大上限，供参考。
      OnSetCartKD_B(K, D, 2);
      OnSetImpType_B(2);
    }
    OnSetSend();
    usleep(100000);
  }
  
  /// set maximum speed and acceleration
  OnClearSet();
  OnSetJointLmt_B(40, 10) ;
  OnSetSend();
  usleep(100000);
}

hardware_interface::CallbackReturn MarvinHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Configuring TJ2 Hardware Interface...");


  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MarvinHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Cleaning up TJ2 Hardware Interface...");
  
  if (hardware_connected_) {
    disconnectFromHardware();
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

void MarvinHardware::contains_gripper()
{
  int joint_index = 0;
  for (const auto& joint : info_.joints) {
        // 检查关节名称中是否包含 gripper 或 hand
        std::string joint_name_lower = joint.name;
        std::transform(joint_name_lower.begin(), joint_name_lower.end(), 
                      joint_name_lower.begin(), ::tolower);
        
        if (joint_name_lower.find("gripper") != std::string::npos || 
            joint_name_lower.find("hand") != std::string::npos) {
            // 这是夹爪关节
            has_gripper_ = true;
            gripper_joint_name_.push_back(joint.name);
            gripper_joint_index_ = joint_index;
            RCLCPP_INFO(get_node()->get_logger(), "Detected gripper joint: %s (index %d)", 
                       joint.name.c_str(), gripper_joint_index_);
        } else {
            // 这是机械臂关节gripper
            joint_names_.push_back(joint.name);
        }
        joint_index++;
  }
  RCLCPP_INFO(get_node()->get_logger(), "Detected gripper joint: %d ", gripper_joint_name_.size());
}

hardware_interface::CallbackReturn MarvinHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Activating TJ2 Hardware Interface...");
  simulation_mode_ = false;
  if (simulation_mode_) {
    RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Running in simulation mode");
    simulation_active_ = true;
    
    // Initialize simulation states
    for (size_t i = 0; i < hw_position_states_.size(); i++) {
      hw_position_states_[i] = 0.0;
      hw_velocity_states_[i] = 0.0;
      hw_effort_states_[i] = 0.0;
      hw_position_commands_[i] = 0.0;
      hw_velocity_commands_[i] = 0.0;
    }
    
  } else {
    RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Running in real hardware mode");
    // Connect to real hardware
    if (!connectToHardware()) {
      RCLCPP_ERROR(rclcpp::get_logger("MarvinHardware"), "Failed to connect to TJ2 hardware");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Read initial joint states from hardware
    if (!readFromHardware(robot_arm_left_right_, true)) {
      RCLCPP_ERROR(rclcpp::get_logger("MarvinHardware"), "Failed to read initial joint states from hardware");
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Initialize commands with current positions
    for (size_t i = 0; i < hw_position_commands_.size(); i++) {
      hw_position_commands_[i] = hw_position_states_[i];
      hw_velocity_commands_[i] = hw_velocity_states_[i];
      // RCLCPP_ERROR(rclcpp::get_logger("MarvinHardware"), "initia position ...  %f", hw_velocity_commands_[i]);
    }
  }
  OnClearSet();
  OnLogOff();
  OnLocalLogOff();
  OnSetSend();
  usleep(100000);
  

  if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::LEFT_ARM))
  {
    setLeftArmCtrl();
  }
  else if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::RIGHT_ARM))
  {
    setRightArmCtrl();
  }
  else if(robot_arm_left_right_ == static_cast<int>(RobotArmConfig::DUAL_ARM))
  {
    setLeftArmCtrl();
    setRightArmCtrl();
  }
  
  OnGetBuf(&frame_data_);
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "current state of A arm:%d\n",frame_data_.m_State[0].m_CurState);
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "cmd state of A arm:%d\n",frame_data_.m_State[0].m_CmdState);
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "error code of A arms:%d\n",frame_data_.m_State[0].m_ERRCode);
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "cmd of vel and acc:%d %d\n",frame_data_.m_In[0].m_Joint_Vel_Ratio,frame_data_.m_In[0].m_Joint_Acc_Ratio);
  
  // connect to gripper
  if (has_gripper_)
  {
    bool gripper_connected = connect_gripper();
    if (gripper_connected)
    {
      RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Gripper Connected");
      // start gripper control thread
      gripper_ctrl_thread_ = std::thread(&MarvinHardware::gripper_callback, this);
      gripper_ctrl_thread_.detach();
      std::thread recv_thread(&MarvinHardware::recv_thread_func, this);
      recv_thread.detach();
      // read onceon_activateon_activate
      // int cur_pos_status = 0;
      // int cur_speed_status = 0;
      // int cur_effort_status = 0;
      // bool success = ModbusGripper::getStatus(cur_effort_status, cur_speed_status, cur_pos_status, OnClearChDataA);
      // RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "gripper read position %d", cur_pos_status);
      // gripper_position_ = (9000 - cur_pos_status) / 9000.0;
      // gripper_position_ = 0.0;
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("MarvinHardware"), "Gripper Not Connected");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "TJ2 Hardware Interface activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

void MarvinHardware::gripper_callback()
{
  while(hardware_connected_)
  {
    // OnClearSet();
    // if(!gripper_stopped_)
    // {
    //   int cur_pos_status = 0;
    //   int cur_speed_status = 0;
    //   int cur_effort_status = 0;
    //   bool success = ModbusGripper::getStatus(cur_effort_status, cur_speed_status, cur_pos_status, OnClearChDataA);
    //   RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "gripper read position %d", cur_pos_status);
    //   gripper_position_ = (9000 - cur_pos_status) / 9000.0;
    //   RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "gripper position %f", gripper_position_);
    //   if (gripper_position_ == last_gripper_position_)
    //   {
    //     gripper_stopped_ = true;
    //   }
    //   else
    //   {
    //     last_gripper_position_ = gripper_position_;
    //   }
    // }
    for(int i =0; i < gripper_stopped_.size(); i++)
    {
      if(!gripper_stopped_[i])
      {
        //RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "gripper write position %f", step_size_);
        gripper_position_[i] = gripper_position_[i] + step_size_[i];
        //RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "gripper write position %f", gripper_position_);
        //RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "gripper write position %f", last_gripper_command_);
        // RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "gripper write position %d", last_gripper_command_ == gripper_position_);
        if (abs(gripper_position_[i]- last_gripper_command_[i]) < 0.001)
        {
          gripper_stopped_[i] = true;
          step_size_[i] = 0.0;
        }
      }

    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    /// write when the gripper commannd is different
    bool success;
    for(int i =0; i < gripper_stopped_.size(); i++)
    {
      if (last_gripper_command_[i] != gripper_position_command_[i])
      {
        RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "gripper %d write position %f", i, gripper_position_command_[i]);
        // write commands to gripper
        int cur_pos_set = 255 - floor(255 / 0.038373 * gripper_position_command_[i]);
        int cur_speed_set = 100;
        int cur_effort_set = 100;
        RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "gripper write position %d", cur_pos_set);
        success = gripper_ptr_[i]->move_gripper(cur_effort_set, cur_speed_set, cur_pos_set);
        last_gripper_command_[i] = gripper_position_command_[i];
        gripper_stopped_[i]= false;
        step_size_[i]= (last_gripper_command_[i] - gripper_position_[i]) / 10.0;
      }
    }
    // OnSetSend();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

bool MarvinHardware::connect_gripper()
{
   bool result = true;
   for(int i =0; i < gripper_ptr_.size(); i++)
   {
      result = result && gripper_ptr_[i]->initialize();
   }
   return result;
}

hardware_interface::CallbackReturn MarvinHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Deactivating TJ2 Hardware Interface...");

  simulation_active_ = false;
  OnClearSet();
  if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::LEFT_ARM))
  {
    OnSetDragSpace_A(0);
    OnSetTargetState_A(0);
    
  }
  else if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::RIGHT_ARM))
  {
    OnSetTargetState_B(0);
  }
  else if(robot_arm_left_right_ == static_cast<int>(RobotArmConfig::DUAL_ARM))
  {
    OnSetTargetState_A(0);
    OnSetTargetState_B(0);
  }
  
  OnSetSend();
  usleep(100000);

  if (hardware_connected_) {
    disconnectFromHardware();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MarvinHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Shutting down TJ2 Hardware Interface...");
  
  if (hardware_connected_) {
    disconnectFromHardware();
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MarvinHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_ERROR(rclcpp::get_logger("MarvinHardware"), "Error in TJ2 Hardware Interface");
  
  // Attempt to safely disconnect from hardware
  if (hardware_connected_) {
    disconnectFromHardware();
  }
  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> MarvinHardware::export_state_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("MarvinHardware"), "Exporting state interfacesExporting for %zu joints", info_.joints.size());
  std::vector<hardware_interface::StateInterface> state_interfaces;
  int joint_name_sz = joint_names_.size();
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Exporting state interfaces for %zu joints", joint_name_sz);
 
  for (size_t i = 0; i < joint_name_sz; i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_position_states_[i]));
    
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_velocity_states_[i]));
    
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_EFFORT,
        &hw_effort_states_[i]));
  }
  if (has_gripper_)
  {
    for(int i =0; i < gripper_joint_name_.size(); i++)
    {
      
      state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        gripper_joint_name_[i],
        hardware_interface::HW_IF_POSITION,
        &gripper_position_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        gripper_joint_name_[i],
        hardware_interface::HW_IF_VELOCITY,
        &gripper_velocity_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        gripper_joint_name_[i],
        hardware_interface::HW_IF_EFFORT,
        &gripper_effort_[i]));
    }
    
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MarvinHardware::export_command_interfaces()
{
  RCLCPP_DEBUG(rclcpp::get_logger("MarvinHardware"), "Exporting command interfaces for %zu joints", info_.joints.size());

  std::vector<hardware_interface::CommandInterface> command_interfaces;
  int joint_name_sz = joint_names_.size();
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Exporting command  interfaces for %zu joints", joint_name_sz);
  for (size_t i = 0; i < joint_name_sz; i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_POSITION,
        &hw_position_commands_[i]));
    
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name,
        hardware_interface::HW_IF_VELOCITY,
        &hw_velocity_commands_[i]));
  }

  if (has_gripper_)
  {
    for(int i =0; i < gripper_joint_name_.size(); i++)
    {

      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          gripper_joint_name_[i],
          hardware_interface::HW_IF_POSITION,
          &gripper_position_command_[i]));
    }
  }

  return command_interfaces;
}

hardware_interface::return_type MarvinHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // if (simulation_active_) {
  //   simulateHardware(period);
  //   return hardware_interface::return_type::OK;
  // }

  if (!hardware_connected_) {
    // RCLCPP_ERROR_THROTTLE(
    //   rclcpp::get_logger("MarvinHardware"), 
    //   *std::make_shared<rclcpp::Clock>(), 5000, 
    //   "Not connected to hardware");
    return hardware_interface::return_type::ERROR;
  }

  if (!readFromHardware(robot_arm_left_right_, false)) {
    // RCLCPP_ERROR_THROTTLE(
    //   rclcpp::get_logger("MarvinHardware"), 
    //   *std::make_shared<rclcpp::Clock>(), 5000, 
    //   "Failed to read from hardware");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MarvinHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // if (simulation_active_) {
  //   // In simulation, commands are handled in the read method
  //   return hardware_interface::return_type::OK;
  // }

  if (!hardware_connected_) {
    return hardware_interface::return_type::ERROR;
  }
  /// convert rad to degree
  // RCLCPP_ERROR(rclcpp::get_logger("MarvinHardware"), "write command ...  %f", hw_position_commands_[2]);
  std::vector<double> hw_commands;
  for(int i =0; i < hw_position_commands_.size(); i++)
  {
    hw_commands.push_back(radToDegree(hw_position_commands_[i]));
  }
  // RCLCPP_ERROR(rclcpp::get_logger("MarvinHardware"), "write command deg ...  %f", hw_position_commands_[2]);
  // Enforce joint limits before sending commands
  // enforceJointLimits();  

  if (!writeToHardware(robot_arm_left_right_, hw_commands)) {
    // RCLCPP_ERROR_THROTTLE(
    //   rclcpp::get_logger("MarvinHardware"),
    //   *clock_, 5000,
    //   "Failed to write to hardware");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}
bool MarvinHardware::recv_thread_func() {
    long ch = 2;                    
    unsigned char data_buf[256] = {0};
    char hex_str[512];

    while (hardware_connected_) {
        if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::LEFT_ARM))
        {
          int size = OnGetChDataA(data_buf, &ch);  
          // if (size > 0 && ch == 2) {
          //     hex_to_str(data_buf, size, hex_str, sizeof(hex_str));
          //     printf("接收字节数: %d, 通道: %ld, HEX: %s\n", size, ch, hex_str);
          // }
        }
        else if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::RIGHT_ARM))
        {
          int size = OnGetChDataB(data_buf, &ch);  
        }
        else if(robot_arm_left_right_ == static_cast<int>(RobotArmConfig::DUAL_ARM))
        {
          int size = OnGetChDataA(data_buf, &ch);
          size = OnGetChDataB(data_buf, &ch);
        }
        usleep(200 * 1000); 
    }
}

bool MarvinHardware::connectToHardware()
{
  device_ip_ = "192.168.1.190";
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Connecting to TJ2 at %s:%d", device_ip_.c_str(), device_port_);
  
  // TODO: Implement actual Dobot connection using Dobot SDK
  // Example:
  unsigned char octet1;
  unsigned char octet2;
  unsigned char octet3;
  unsigned char octet4;
  
  // hardware_connected_ = OnLinkTo(octet1,octet2,octet3,octet4) == true;
  hardware_connected_ = OnLinkTo(192,168,1,190);
      
  
  // Simulate connection for demonstration
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // hardware_connected_ = true;
  if (hardware_connected_)
  {
    RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Successfully connected to TJ2 hardware");
    usleep(100000);
    OnClearSet();
    OnClearErr_A();
    OnClearErr_B();
    OnSetSend();
    usleep(100000);
  }
  else
  {
    RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Failed to connect to TJ2 hardware");
  }
  
  return hardware_connected_;
}

void MarvinHardware::disconnectFromHardware()
{
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Disconnecting from Dobot CR5 hardware");
  
  // TODO: Implement actual Dobot disconnection
  OnRelease();
  
  hardware_connected_ = false;
  RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Disconnected from Dobot CR5 hardware");
}

bool MarvinHardware::readFromHardware(int robot_arm_left_right_, bool initial_frame)
{
  
  OnGetBuf(&frame_data_);
    // RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Reading from hardware interface ... %d", frame_data_.m_Out[robot_arm_left_right_].m_OutFrameSerial);
  if (initial_frame)
  {
    previous_message_frame_ = frame_data_.m_Out[robot_arm_left_right_].m_OutFrameSerial;
  }
  if (robot_arm_left_right_ == static_cast<int>(RobotArmConfig::LEFT_ARM)|| robot_arm_left_right_ == static_cast<int>(RobotArmConfig::RIGHT_ARM))
  {
    for (size_t i = 0; i < 7; i++) {
      hw_position_states_[i] = degreeToRad(frame_data_.m_Out[robot_arm_left_right_].m_FB_Joint_Pos[i]);
      hw_velocity_states_[i] = frame_data_.m_Out[robot_arm_left_right_].m_FB_Joint_Vel[i];
      hw_effort_states_[i] = frame_data_.m_Out[robot_arm_left_right_].m_FB_Joint_SToq[i];
    }
  }
  else if(robot_arm_left_right_ == static_cast<int>(RobotArmConfig::DUAL_ARM))
  {
    for (size_t i = 0; i < 7; i++) {
      hw_position_states_[i] = degreeToRad(frame_data_.m_Out[static_cast<int>(RobotArmConfig::LEFT_ARM)].m_FB_Joint_Pos[i]);
      hw_velocity_states_[i] = frame_data_.m_Out[static_cast<int>(RobotArmConfig::LEFT_ARM)].m_FB_Joint_Vel[i];
      hw_effort_states_[i] = frame_data_.m_Out[static_cast<int>(RobotArmConfig::LEFT_ARM)].m_FB_Joint_SToq[i];
    }

    for (size_t i = 0; i < 7; i++) {
      hw_position_states_[i + 7] = degreeToRad(frame_data_.m_Out[static_cast<int>(RobotArmConfig::RIGHT_ARM)].m_FB_Joint_Pos[i]);
      hw_velocity_states_[i + 7] = frame_data_.m_Out[static_cast<int>(RobotArmConfig::RIGHT_ARM)].m_FB_Joint_Vel[i];
      hw_effort_states_[i + 7] = frame_data_.m_Out[static_cast<int>(RobotArmConfig::RIGHT_ARM)].m_FB_Joint_SToq[i];
    }
  }
  previous_message_frame_ = frame_data_.m_Out[robot_arm_left_right_].m_OutFrameSerial;

    if (previous_message_frame_ - frame_data_.m_Out[robot_arm_left_right_].m_OutFrameSerial < 2)
    {
      return true;
    }
   else
   {
        RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "Missing more than 2 frames");
        return true;
   }
}

bool MarvinHardware::writeToHardware(int robot_arm_left_right, std::vector<double> & hw_commands)
{
  // TODO: Implement actual joint command sending to Dobot
  bool result = true;
  OnClearSet(); 
  if (robot_arm_left_right == static_cast<int>(RobotArmConfig::LEFT_ARM))
  {
    result = OnSetJointCmdPos_A(hw_commands.data());
  }
  else if (robot_arm_left_right == static_cast<int>(RobotArmConfig::RIGHT_ARM))
  {
    result = OnSetJointCmdPos_B(hw_commands.data());
  }
  else if (robot_arm_left_right == static_cast<int>(RobotArmConfig::DUAL_ARM))
  {
    result = OnSetJointCmdPos_A(hw_commands.data());
    // RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "write to left arm %d", result);
    result = result && OnSetJointCmdPos_B(hw_commands.data() + 7);
    // RCLCPP_INFO(rclcpp::get_logger("MarvinHardware"), "write to right arm %d", result);
  }
  OnSetSend();
  return result;
}

void MarvinHardware::simulateHardware(const rclcpp::Duration & period)
{
  // Simple simulation: move toward commanded position with velocity limits
  for (size_t i = 0; i < hw_position_states_.size(); i++) {
    double position_error = hw_position_commands_[i] - hw_position_states_[i];
    double max_velocity_change = velocity_limits_[i] * period.seconds();
    
    // Calculate desired velocity
    double desired_velocity = std::copysign(
      std::min(std::abs(position_error) / period.seconds(), velocity_limits_[i]),
      position_error
    );
    
    // Apply velocity limits
    double velocity_change = desired_velocity - hw_velocity_states_[i];
    if (std::abs(velocity_change) > max_velocity_change) {
      velocity_change = std::copysign(max_velocity_change, velocity_change);
    }
    
    hw_velocity_states_[i] += velocity_change;
    hw_position_states_[i] += hw_velocity_states_[i] * period.seconds();
    
    // Simulate effort based on acceleration
    hw_effort_states_[i] = velocity_change / period.seconds() * 0.1;
  }
}

void MarvinHardware::enforceJointLimits()
{
  if (hw_position_commands_.size() != position_lower_limits_.size() || 
      hw_position_commands_.size() != position_upper_limits_.size()) {
    RCLCPP_ERROR(*logger_, "Array size mismatch in enforceJointLimits!");
    return;
  }

  for (size_t i = 0; i < hw_position_commands_.size(); i++) {
    if (hw_position_commands_[i] < position_lower_limits_[i]) {
      hw_position_commands_[i] = position_lower_limits_[i];
      RCLCPP_WARN_THROTTLE(
        *logger_, 
        *clock_, 10000,  // Use the class member clock
        "Joint %s position command (%.3f) below lower limit (%.3f)", 
        info_.joints[i].name.c_str(), hw_position_commands_[i], position_lower_limits_[i]);
    } else if (hw_position_commands_[i] > position_upper_limits_[i]) {
      hw_position_commands_[i] = position_upper_limits_[i];
      RCLCPP_WARN_THROTTLE(
        *logger_, 
        *clock_, 10000,
        "Joint %s position command (%.3f) above upper limit (%.3f)", 
        info_.joints[i].name.c_str(), hw_position_commands_[i], position_upper_limits_[i]);
    }
    
    // Add safety check for velocity arrays too
    if (i < hw_velocity_commands_.size() && i < velocity_limits_.size() && 
        std::abs(hw_velocity_commands_[i]) > velocity_limits_[i]) {
      hw_velocity_commands_[i] = std::copysign(velocity_limits_[i], hw_velocity_commands_[i]);
    }
  }
}

bool MarvinHardware::initializeJointLimits()
{
  // position_lower_limits_.resize(info_.joints.size());
  // position_upper_limits_.resize(info_.joints.size());
  // velocity_limits_.resize(info_.joints.size());
  // effort_limits_.resize(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); i++) {
    const auto & joint = info_.joints[i];
    
    // Parse position limits
    for (const auto & command_interface : joint.command_interfaces) {
      if (command_interface.name == hardware_interface::HW_IF_POSITION) {
        if (!command_interface.min.empty()) {
          position_lower_limits_[i] = std::stod(command_interface.min);
        }
        if (!command_interface.max.empty()) {
          position_upper_limits_[i] = std::stod(command_interface.max);
        }
      } else if (command_interface.name == hardware_interface::HW_IF_VELOCITY) {
        if (!command_interface.max.empty()) {
          velocity_limits_[i] = std::stod(command_interface.max);
        }
      }
    }
    
    // Parse effort limits from joint limits in URDF
    effort_limits_[i] = std::stod(joint.command_interfaces[0].max);
    
    RCLCPP_DEBUG(
      rclcpp::get_logger("MarvinHardware"),
      "Joint %s: pos=[%.3f, %.3f], vel_limit=%.3f, effort_limit=%.3f",
      joint.name.c_str(), position_lower_limits_[i], position_upper_limits_[i],
      velocity_limits_[i], effort_limits_[i]);
  }

  return true;
}

void MarvinHardware::logJointStates()
{
  std::stringstream ss;
  ss << "Joint States - ";
  for (size_t i = 0; i < hw_position_states_.size(); i++) {
    ss << info_.joints[i].name << ": pos=" << std::fixed << std::setprecision(3) << hw_position_states_[i]
       << ", vel=" << hw_velocity_states_[i] << ", cmd=" << hw_position_commands_[i];
    if (i < hw_position_states_.size() - 1) ss << " | ";
  }
  RCLCPP_DEBUG(rclcpp::get_logger("MarvinHardware"), "%s", ss.str().c_str());
}

}  // namespace marvin_ros2_control


PLUGINLIB_EXPORT_CLASS(marvin_ros2_control::MarvinHardware, hardware_interface::SystemInterface)

