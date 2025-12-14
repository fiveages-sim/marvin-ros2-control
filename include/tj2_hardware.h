#ifndef MARVIN_HARDWARE__MARVIN_HARDWARE_HPP_
#define MARVIN_HARDWARE__MARVIN_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <sstream>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "MarvinSDK.h"
#include <cmath>
#include "gripper_control.h"

namespace marvin_ros2_control
{
    enum class RobotArmConfig {
      LEFT_ARM = 0,
      RIGHT_ARM = 1,
      DUAL_ARM = 2
    };

    enum class RobotCtrlMode{
      POSITION = 2,
      JOINT_IMPEDANCE = 3,
      CART_IMPEDANCE = 4,
    };
  
class MarvinHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MarvinHardware)

  // Destructor
  virtual ~MarvinHardware();

  // Hardware interface lifecycle methods
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  // Hardware interface methods
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Hardware parameters
  std::shared_ptr<rclcpp::Node> node_;
  std::string device_ip_;
  int device_port_;
  double simulation_mode_;
  double read_timeout_;
  double write_timeout_;
  int  robot_arm_left_right_;
  int  robot_ctrl_mode_;
  int  previous_message_frame_;
  DCSS frame_data_;
  std::shared_ptr<rclcpp::Logger> logger_;
  std::shared_ptr<rclcpp::Clock> clock_;
  // Joint data storage
  std::vector<double> hw_position_commands_;
  std::vector<double> hw_velocity_commands_;
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_effort_states_;

  // Joint limits from URDF
  std::vector<double> position_lower_limits_;
  std::vector<double> position_upper_limits_;
  std::vector<double> velocity_limits_;
  std::vector<double> effort_limits_;

  // Connection status
  bool hardware_connected_;
  bool simulation_active_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  std::vector<std::string> joint_names_;
  // Dobot communication handle (placeholder)
  // void* dobot_handle_;

  // Helper methods
  bool connectToHardware();
  void disconnectFromHardware();
  bool readFromHardware(int robot_arm_left_right_, bool initial_frame);
  bool writeToHardware(int robot_arm_left_right_, std::vector<double> & hw_commands);
  void simulateHardware(const rclcpp::Duration & period);
  void enforceJointLimits();
  bool initializeJointLimits();
  void logJointStates();
  void setLeftArmCtrl();
  void setRightArmCtrl();
  rcl_interfaces::msg::SetParametersResult paramCallback(const std::vector<rclcpp::Parameter> & params);
  void applyRobotConfiguration(int mode, int arm_side, int drag_mode, int cart_type,
                              double max_joint_speed, double max_joint_acceleration,
                              const std::vector<double>& joint_k_gains,
                              const std::vector<double>& joint_d_gains,
                              const std::vector<double>& cart_k_gains,
                              const std::vector<double>& cart_d_gains);
  
  double degreeToRad(double degree) {
      return degree * M_PI / 180.0;
  };

  double radToDegree(double rad) {
    return rad * 180.0 / M_PI;
  };
  
  //
  // 夹爪参数
  bool                                  has_gripper_;
  std::vector<std::string>               gripper_joint_name_;
  size_t                        gripper_joint_index_;
  std::vector<double>                        last_gripper_position_;
  std::vector<double>                     gripper_position_; 
  std::vector<double>                           gripper_velocity_;
  std::vector<double>                          gripper_effort_;
  std::vector<double>                           gripper_position_command_;
  std::vector<double>                         last_gripper_command_;
  std::vector<bool>                          gripper_stopped_ ;
  bool                             gripper_initilized_;
  void                              contains_gripper();
  std::vector<double>                           step_size_;
  std::thread gripper_ctrl_thread_;
  std::vector<std::unique_ptr<marvin_ros2_control::ModbusGripper>> gripper_ptr_;
  void gripper_callback();
  bool recv_thread_func();
  bool connect_gripper();
  void disconnect_gripper();

  void splitIPToCharArrays(const char* ipStr, char octet1[4], char octet2[4], char octet3[4], char octet4[4]) {
        sscanf(ipStr, "%3[^.].%3[^.].%3[^.].%3[^.]", octet1, octet2, octet3, octet4);
    }
};

}  // namespace marvin_ros2_control

#endif  // MARVIN_HARDWARE__MARVIN_HARDWARE_HPP_