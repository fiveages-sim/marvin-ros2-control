#ifndef MARVIN_HARDWARE__MARVIN_HARDWARE_HPP_
#define MARVIN_HARDWARE__MARVIN_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "MarvinSDK.h"
#include <cmath>
#include "marvin_ros2_control/tool/grippers/modbus_gripper.h"
#include "marvin_ros2_control/tool/hands/modbus_hand.h"
#include "gripper_hardware_common/GripperBase.h"

namespace marvin_ros2_control
{
    // Arm configuration constants
    constexpr int ARM_LEFT = 0;
    constexpr int ARM_RIGHT = 1;
    constexpr int ARM_DUAL = 2;
  
    class MarvinHardware : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(MarvinHardware)

        // Destructor
        virtual ~MarvinHardware(){};

        // Hardware interface lifecycle methods
        hardware_interface::CallbackReturn
        on_init(const hardware_interface::HardwareComponentInterfaceParams& params) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
        hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

        // Hardware interface methods
        std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;
        hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
        hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
        rclcpp::Logger get_logger() const 
        { 
            return logger_.value();  // Safe to use value() since logger_ is initialized in on_init()
        }
        // Logger (initialized in on_init using get_node()->get_logger())
        std::optional<rclcpp::Logger> logger_;

        // Hardware parameters
        std::shared_ptr<rclcpp::Node> node_;
        std::string device_ip_;
        int device_port_ = 8080;
        std::string robot_arm_config_;  // "LEFT", "RIGHT", "DUAL"
        int robot_arm_index_ = 0;       // 0=LEFT, 1=RIGHT, 2=DUAL (在初始化时设定)
        std::string robot_ctrl_mode_;   // "POSITION", "JOINT_IMPEDANCE", "CART_IMPEDANCE"
        int previous_message_frame_ = 0;
        DCSS frame_data_;

        // Control parameters
        // 这些成员变量是 ROS2 参数的缓存值，命名与参数保持一致，减少歧义
        double max_joint_speed_;
        double max_joint_acceleration_;
        std::vector<double> joint_k_gains_;
        std::vector<double> joint_d_gains_;
        std::vector<double> cart_k_gains_;
        std::vector<double> cart_d_gains_;
        std::vector<double> leftdynParam_;
        std::vector<double> leftkineParam_;
        std::vector<double> rightdynParam_;
        std::vector<double> rightkineParam_;
        // Joint data storage
        std::vector<double> hw_position_commands_;
        std::vector<double> hw_velocity_commands_;
        std::vector<double> hw_position_states_;
        std::vector<double> hw_velocity_states_;
        std::vector<double> hw_effort_states_;
        // 复用的写入缓存（避免在 write() 控制回路中频繁分配/扩容导致抖动）
        std::vector<double> hw_commands_deg_buffer_;

        // Joint limits from URDF
        std::vector<double> position_lower_limits_;
        std::vector<double> position_upper_limits_;
        std::vector<double> velocity_limits_;
        std::vector<double> effort_limits_;

        // Connection status
        bool hardware_connected_;
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
        std::vector<std::string> joint_names_;

        // Helper methods
        bool connectToHardware();
        void disconnectFromHardware();
        bool readFromHardware(bool initial_frame);
        bool writeToHardware(std::vector<double>& hw_commands);
        void setArmCtrlInternal(int arm_index);
                
        // Helper method to create tool (gripper or hand) based on type
        // tool_index: 0 for left hand (in dual arm) or single left arm, 1 for right hand (in dual arm) or single right arm
        std::unique_ptr<gripper_hardware_common::GripperBase> createTool(
            Clear485Func clear_485, 
            Send485Func send_485,
            GetChDataFunc get_ch_data,
            size_t tool_index = 0);
        void set_tool_parameters();

        static double degreeToRad(const double degree)
        {                
            return degree * M_PI / 180.0;
        }

        static double radToDegree(const double rad)
        {
            return rad * 180.0 / M_PI;
        }
        
        // 辅助方法：获取节点参数，如果不存在则自动声明
        template<typename T>
        T get_node_param(const std::string& name, const T& default_val)
        {
            if (!node_->has_parameter(name)) {
                node_->declare_parameter<T>(name, default_val);
            }
            // rclcpp::Parameter 使用 get_value<T>() 获取强类型值
            return node_->get_parameter(name).get_value<T>();
        }
        
        rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & params);
        void applyRobotConfiguration(int mode, int drag_mode, int cart_type,
                                    double max_joint_speed, double max_joint_acceleration,
                                    const std::vector<double>& joint_k_gains,
                                    const std::vector<double>& joint_d_gains,
                                    const std::vector<double>& cart_k_gains,
                                    const std::vector<double>& cart_d_gains);
        void declare_node_parameters();

        
        // Gripper parameters
        std::string gripper_type_;
        double gripper_torque_scale_ = 1.0;  // Torque scaling factor (0.0-1.0, default: 1.0)
        bool has_gripper_ = false;
        std::vector<std::string> gripper_joint_name_;
        size_t gripper_joint_index_ = 0;
        std::vector<double> last_gripper_position_;
        std::vector<double> gripper_position_;
        std::vector<double> gripper_velocity_;
        std::vector<double> gripper_effort_;
        std::vector<double> gripper_position_command_;
        std::vector<double> last_gripper_command_;
        std::vector<bool> gripper_stopped_;
        bool gripper_initilized_ = false;
        void contains_gripper();
        std::vector<double> step_size_;
        std::thread gripper_ctrl_thread_;
        std::vector<std::unique_ptr<gripper_hardware_common::GripperBase>> tool_ptr_;  // Unified container for hand/gripper
        bool is_hand_ = false;  // Flag to indicate if end effector is a hand (true) or gripper (false) - used for logging only
        void tool_callback();
        bool recv_thread_func();
        void updateGripperState(size_t gripper_idx, double position, int velocity, int torque);
        bool connect_gripper();
        void disconnect_gripper();
        
        // Unified tool access helpers
        size_t toolCount() const { return tool_ptr_.size(); }
        gripper_hardware_common::GripperBase* toolAt(size_t idx) 
        { 
            return (idx < tool_ptr_.size()) ? tool_ptr_[idx].get() : nullptr; 
        }
        const gripper_hardware_common::GripperBase* toolAt(size_t idx) const 
        { 
            return (idx < tool_ptr_.size()) ? tool_ptr_[idx].get() : nullptr; 
        }

    };
} // namespace marvin_ros2_control

#endif  // MARVIN_HARDWARE__MARVIN_HARDWARE_HPP_