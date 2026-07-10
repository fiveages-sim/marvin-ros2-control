#ifndef MARVIN_HARDWARE__MARVIN_HARDWARE_HPP_
#define MARVIN_HARDWARE__MARVIN_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <thread>
#include <array>
#include <atomic>
#include <mutex>
#include <chrono>
#include <cstdint>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "MarvinSDK.h"
#include <cmath>
#include "marvin_ros2_control/tool/grippers/modbus_gripper.h"
#include "marvin_ros2_control/tool/hands/modbus_hand.h"
#include "gripper_hardware_common/GripperBase.h"
#include "marvin_ros2_control/tool/modbus_io.h"
#include "marvin_ros2_control/sensors/kwr75_protocol.h"

namespace marvin_ros2_control
{
    constexpr size_t kMaxTools = 2;

    // Arm configuration constants
    constexpr int ARM_LEFT = 0;
    constexpr int ARM_RIGHT = 1;
    constexpr int ARM_DUAL = 2;

    enum class ToolType { None, Hand, Gripper, Others };

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
        rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr hardware_error_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fsm_command_pub_;
        std::string device_ip_;
        int device_port_ = 8080;
        std::string robot_arm_config_;  // "LEFT", "RIGHT", "DUAL"
        int robot_arm_index_ = 0;       // 0=LEFT, 1=RIGHT, 2=DUAL (在初始化时设定)
        std::string robot_ctrl_mode_;   // "POSITION", "JOINT_IMPEDANCE", "CART_IMPEDANCE", "POWER_OFF"
        std::string last_active_ctrl_mode_ = "POSITION";
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

        // Virtual FT sensor state (filled from external wrench topic)
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr left_wrench_sub_;
        rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr right_wrench_sub_;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr left_wrench_pub_;
        rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr right_wrench_pub_;
        std::mutex wrench_mutex_;
        std::array<double, 6> left_ft_state_{};
        std::array<double, 6> right_ft_state_{};
        std::array<double, 6> single_ft_state_{};  // used for single-arm configs
        Kwr75FtConfig kwr75_ft_config_;
        std::atomic<bool> kwr75_ft_running_{false};
        std::thread kwr75_ft_thread_left_;
        std::thread kwr75_ft_thread_right_;

        void loadKwr75FtConfig();
        void startKwr75FtThreads();
        void stopKwr75FtThreads();
        void kwr75FtThread(int arm_index);
        void applyKwr75Wrench(int arm_index, const std::array<double, 6>& wrench);

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
        bool left_brake_released_ = false;
        bool right_brake_released_ = false;

        // Helper method to create tool (gripper or hand) based on type
        // tool_index: 0 for left hand (in dual arm) or single left arm, 1 for right hand (in dual arm) or single right arm
        std::unique_ptr<gripper_hardware_common::GripperBase> createTool(
            Clear485Func clear_485, 
            Send485Func send_485,
            GetChDataFunc get_ch_data,
            size_t tool_index = 0,
            const std::string& ee_type = "",
            long channel = COM1_CHANNEL);
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
        void syncToolDynamicsFromNodeParams();
        void applyAllToolDynamics(const std::unordered_map<std::string, double>* pending = nullptr);
        /** 0=left, 1=right tool slot; same rule as position-control joint mapping. */
        size_t mappedToolIndexForJoint(size_t k) const;
        bool gripperJointBelongsToTool(size_t k, size_t tool_idx) const;

        
        // Gripper / hand: normalized torque/velocity from ROS params (not HW command interfaces).
        std::string gripper_type_;
        std::string left_ee_type_;
        std::string right_ee_type_;
        long left_ee_channel_ = COM1_CHANNEL;
        long right_ee_channel_ = COM1_CHANNEL;
        double gripper_torque_scale_ = 1.0;  // Torque scaling factor (0.0-1.0, default: 1.0)
        /** If true, enable high-frequency INFO logs for tool (hand/gripper) and Modbus hex dumps. */
        std::vector<double> gripper_effort_command_;   // HW_IF_EFFORT command → normalized torque to tool
        std::vector<double> gripper_velocity_command_; // HW_IF_VELOCITY command → normalized velocity to tool
        bool debug_tool_logs_ = false;
        bool has_gripper_ = false;
        std::vector<std::string> gripper_joint_name_;
        size_t gripper_joint_index_ = 0;
        // For grippers without a "target reached" feedback, infer stop by comparing consecutive frames.
        static constexpr size_t kGripperStableFrameCount = 3;
        static constexpr double kGripperStableEpsilon = 0.001;
        std::vector<double> gripper_previous_position_;
        std::vector<size_t> gripper_stable_count_;
        std::vector<double> gripper_position_;
        std::vector<double> gripper_velocity_;
        std::vector<double> gripper_effort_;
        std::vector<double> gripper_position_command_;
        std::vector<double> last_gripper_command_;
        std::vector<double> last_gripper_effort_ack_;
        std::vector<double> last_gripper_velocity_ack_;
        std::vector<bool> gripper_stopped_;
        void contains_tool();
        bool eeTypeIsHand(const std::string& ee_type) const;
        std::string eeTypeForTool(size_t tool_idx) const;
        bool toolIsHand(size_t tool_idx) const;
        std::vector<std::unique_ptr<gripper_hardware_common::GripperBase>> tool_ptr_;  // Unified container for hand/gripper
        std::vector<bool> tool_is_left_side_;
        std::vector<std::string> tool_ee_types_;
        bool toolUsesLeftChannel(size_t tool_idx) const;
        long toolChannel(size_t tool_idx) const;
        ToolType tool_type_ = ToolType::None;  // Hand, Gripper, Others, or None - determines move_hand vs move_gripper
        const char* toolTypeLogName() const;
        // Single in-flight task per tool: 0=None, 1=Read (waiting response), 2=Write (waiting response)
        std::array<std::atomic<int>, kMaxTools> in_flight_type_{};
        std::array<std::vector<double>, kMaxTools> in_flight_write_command_;
        // One initial read per tool; after that only read when tool is not stopped
        std::array<bool, kMaxTools> tool_initial_read_done_{};
        /** True if this tool failed initial read; skip all read/write polling for this side. */
        std::array<bool, kMaxTools> tool_init_failed_{};
        /** True if we have received and parsed at least one valid status (read) response for this tool. */
        std::array<std::atomic<bool>, kMaxTools> tool_has_valid_state_{};
        // Hand: current frame same as previous for N consecutive reads -> steady state, stop read polling until next write
        static constexpr size_t kHandStableFrameCount = 5;
        std::array<std::vector<double>, kMaxTools> hand_previous_position_;
        std::array<size_t, kMaxTools> hand_stable_count_{};
        std::array<bool, kMaxTools> hand_stabilized_{};
        std::vector<std::thread> gripper_ctrl_threads_;
        /** Async mode: one recv thread per tool, only polls get485 and dispatches to processToolResponse / write ack. */
        std::vector<std::thread> tool_recv_threads_;
        /** If true, use async tool comm: recv thread(s) + send-only tool_callback_for_tool_async (see marvin_*_init_example). */
        bool use_async_tool_comm_ = true;
        /** If true, initialize end-effector (hand/gripper) on activate. If false, skip tool initialize/reads/threads. */
        bool init_tool_on_startup_ = true;
        /** Async heartbeat: send periodic read (getStatus) and require read response frames to declare link healthy. */
        std::array<std::atomic<std::int64_t>, kMaxTools> tool_hb_start_ms_{};
        std::array<std::atomic<std::int64_t>, kMaxTools> tool_hb_tx_ms_{};
        std::array<std::atomic<std::int64_t>, kMaxTools> tool_hb_last_rx_ms_{};
        std::array<std::atomic<bool>, kMaxTools> tool_hb_offline_reported_{};

        // --- Async per-cycle request/response pairing & timeout accounting ---
        // Fully decoupled design: sender thread only marks a frame pending; the recv
        // thread (20ms cadence) clears the pending flag when it parses a reply.
        // At the START of the next control cycle, if pending is still set, it means the
        // recv thread did not observe a reply within ~n chances (n = ctrl_period/recv_period),
        // so the previous frame is counted as a timeout. Zero synchronization cost on
        // the sender path besides atomic flag flips.
        std::array<std::atomic<bool>, kMaxTools> tool_reply_pending_{};     // true = sent, awaiting reply
        std::array<std::atomic<int>, kMaxTools> tool_reply_kind_{};         // 0=none,1=read,2=write (for logging)
        std::array<std::atomic<std::uint64_t>, kMaxTools> tool_tx_total_{}; // total frames sent
        std::array<std::atomic<std::uint64_t>, kMaxTools> tool_rx_total_{}; // frames answered (pending cleared by recv)
        std::array<std::atomic<std::uint64_t>, kMaxTools> tool_timeout_count_{}; // frames never answered within cycle
        // Last reported snapshot of timeout counters (for periodic summary diffs).
        std::array<std::uint64_t, kMaxTools> tool_last_reported_timeout_{};

        void tool_callback_for_tool(size_t tool_idx);
        /** Async: recv thread for one tool; only calls get_ch_data and processes responses (no sending). */
        void tool_recv_thread_func(size_t tool_idx);
        /** Async: same loop as tool_callback_for_tool but only sends (getStatus/move); no blocking receive. */
        void tool_callback_for_tool_async(size_t tool_idx);
        /** Read once from channel, copy to data_buf, return byte count or 0. */
        long receiveToolResponse(unsigned char* data_buf, size_t buf_size, GetChDataFunc get_ch_data, long channel);
        void processToolResponse(const unsigned char* data_buf, size_t size, size_t gripper_idx);
        bool isToolStateCloseToCommand(size_t tool_idx, double threshold);
        /** True if tool is stopped (hand: at command and stabilized; gripper: at target and stopped). */
        bool isToolStopped(size_t tool_idx);
        /** Sync read: one getStatus(), wait elapsed_time_for_poll ms, then read and parse. */
        bool readToolStatusSync(size_t tool_idx, int elapsed_time_for_poll);
        /** Hand init: accept current speed/force commands as acknowledged to avoid a startup write. */
        void syncHandDynamicsAckToCommand(size_t tool_idx);
        /** Sync write: send write_cmd, wait wait_ms, read ack and update state. */
        bool writeToolStatusSync(size_t tool_idx, const std::vector<double>& write_cmd, int wait_ms = 5);
        /** True if a new write command should be sent for this tool (position changed); fills write_cmd_out. */
        bool shouldSendToolCommand(size_t tool_idx, std::vector<double>& write_cmd_out);
        /** If a write is in flight for this tool, try one non-blocking receive; on success update last and clear in_flight. */
        void tryConsumeWriteAck(size_t tool_idx);
        /** True if data_buf looks like Modbus write response (FC 0x10). */
        static bool isModbusWriteAck(const unsigned char* data_buf, size_t size);
        /**
         * @brief Check the previous cycle's pending reply; if still unanswered, count a timeout.
         *
         * Call this at the START of each control cycle BEFORE sending a new frame.
         * It is fully non-blocking: it only inspects the atomic tool_reply_pending_ flag
         * which the recv thread (20ms cadence) clears on a valid reply.
         *
         * @return true if the previous frame timed out (no reply within the cycle).
         */
        bool checkPrevCycleTimeout(size_t tool_idx);
        /** Mark a frame as just-sent (pending reply). Called by sender right after send_485. */
        void markFrameSent(size_t tool_idx, int kind);
        /** Mark a frame as answered. Called by recv thread when it parses a valid reply. */
        void markFrameAnswered(size_t tool_idx);
        /** Sync-mode accounting: update tx/rx/timeout counters immediately from a known send result. */
        void accountSyncFrame(size_t tool_idx, int kind, bool got_reply);
        /** Periodic timeout summary: emits an aggregated WARN when new timeouts occurred since last call. */
        void emitTimeoutSummary(size_t tool_idx);
        /** Apply in-flight write as acknowledged: update last_gripper_command_ and clear in_flight for tool_idx. */
        void applyGripperWriteAckFromInFlight(size_t tool_idx);
        void updateGripperState(size_t gripper_idx, double position, int velocity, int torque);
        bool connect_tool();
        void disconnect_gripper();
        /** One-time initial read per tool during on_activate; updates state and sets tool_initial_read_done_. Returns false if any tool fails to respond. */
        bool doInitialToolReads();
        /** Map tool_idx (0=left/A, 1=right/B) to gripper joint array index. */
        size_t gripperJointIndexForTool(size_t tool_idx) const;

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
