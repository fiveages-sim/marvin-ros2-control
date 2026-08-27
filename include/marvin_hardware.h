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
#include <condition_variable>
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

// SDK include chain (Robot.h → TCPFileClient.h → FileOP.h) leaves #pragma pack(4) active
// without pop, shrinking sizeof() for all types below in this TU vs gripper .cpp files.
#include "MarvinSDK.h"
#pragma pack()
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
        std::string robot_ctrl_mode_;   // "POSITION", "JOINT_IMPEDANCE", "CART_IMPEDANCE", "POWER_OFF", "PD"
        std::string last_active_ctrl_mode_ = "POSITION";
        int previous_message_frame_ = 0;
        DCSS frame_data_;

        // OnClearSet false 暂时只计数并跳过本周期，不触发硬件错误；成功后清零并报告。
        std::uint64_t clear_set_consecutive_false_count_ = 0;
        // OnSetSend 连续失败仍保留阈值保护。
        int main_frame_submit_failure_count_ = 0;
        static constexpr int kMainFrameBusyFatalThreshold = 10;

        // --- SDK 单线程访问约束 ---
        // 底层 SDK 的 clear/send（主帧槽）与通道发送要求单线程串行访问。因此所有 SDK
        // 请求（机械臂主帧、工具控制/状态读取、KWR75 轮询）一律收拢到控制线程 write()
        // 按优先级每周期发送；其它线程（参数回调、生命周期）只置"待执行动作"，由 write()
        // 执行。sdk_access_mutex_ 用于 write() 的 SDK 段与生命周期兜底直发之间互斥
        // （write() 已停转的极端场景），保证 SDK 永不被并发访问。
        std::mutex sdk_access_mutex_;

        // 待执行的主帧动作（同一时刻至多一个；write() 每周期执行一步）
        enum class PendingArmActionKind : std::uint8_t
        {
            kNone = 0,
            kBrake,          // paramCallback：松闸/抱闸（仅在 POWER_OFF 模式允许）
            kPowerOff,       // on_deactivate：目标状态置 0（下使能）
            kEmergencyStop,  // on_error/on_shutdown：抱闸 + 目标状态置 0
        };
        struct PendingArmAction
        {
            PendingArmActionKind kind = PendingArmActionKind::kNone;
            bool left = false;          // 是否更新左臂
            bool right = false;         // 是否更新右臂
            bool brake_release = false; // kBrake：true=松闸(2)，false=抱闸(1)
            int step = 0;               // 已执行步数
            int total_steps = 0;        // 总步数（达此值即完成）
        };
        std::mutex pending_action_mutex_;
        std::condition_variable pending_action_cv_;
        PendingArmAction pending_action_;

        // write() 每周期执行一步待执行动作；返回 true 表示本周期主帧已被动作占用。
        bool tryExecutePendingActionStep();
        // 动作类型可读名（日志用）。
        static const char* pendingActionKindName(PendingArmActionKind k);
        // 其它线程请求执行动作；完成后通过 cv 通知。
        void requestPendingAction(PendingArmAction action);
        // 等待动作完成（wait_ms 超时返回 false，调用方据此走兜底直发）。
        bool waitPendingActionDone(int wait_ms);
        // 兜底：write() 未运行时直接串行执行全部步骤（需先获取 sdk_access_mutex_）。
        void executePendingActionDirect(PendingArmAction action);
        // 单步执行 kEmergencyStop 中"按需抱闸"（OnGetIntPara 检查 + 发送）。
        void engageBrakeIfNeeded(int arm_index);

        // KWR75 轮询线程模式的发送收拢：线程只置挂起标志，由 write() 代为发送。
        std::atomic<bool> kwr75_poll_pending_left_{false};
        std::atomic<bool> kwr75_poll_pending_right_{false};
        void sendPendingKwr75Polls();

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

        // FT sensor state interfaces (left_ft_sensor / right_ft_sensor).
        // Exported only when URDF declares <sensor>=left_ft_sensor or right_ft_sensor
        // (driven by xacro marvin_ft_sensor_interfaces from robot.local.yaml).
        std::mutex ft_state_mutex_;
        std::array<double, 6> left_ft_state_{};
        std::array<double, 6> right_ft_state_{};
        Kwr75FtConfig kwr75_ft_config_;
        Kwr75LockFreeSample left_kwr75_sample_{};
        Kwr75LockFreeSample right_kwr75_sample_{};
        std::atomic<bool> kwr75_ft_running_{false};
        std::thread kwr75_ft_thread_left_;
        std::thread kwr75_ft_thread_right_;

        void loadKwr75FtConfig();
        void startKwr75FtThreads();
        void stopKwr75FtThreads();
        void kwr75FtThread(int arm_index);
        /** Write wrench into exported FT state interfaces only (no topic publish). */
        void applyKwr75Wrench(int arm_index, const std::array<double, 6>& wrench);
        /** When ft_poll_interval_ms<=0: update FT HI from latest COM2 sample each read(). */
        void updateKwr75StateInterfaces();
        bool kwr75UsesPollThread() const { return kwr75_ft_config_.poll_interval_ms > 0; }

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
        std::vector<double> gripper_effort_command_;   // HW_IF_EFFORT command → normalized torque to tool
        std::vector<double> gripper_velocity_command_; // HW_IF_VELOCITY command → normalized velocity to tool
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
        /** If true, tool send runs in write(); OnGetChData runs in hardware read(). */
        bool use_async_tool_comm_ = true;
        /** If true, initialize end-effector (hand/gripper) on activate. If false, skip tool initialize/reads/threads. */
        bool init_tool_on_startup_ = true;
        /** Async heartbeat: send periodic read (getStatus) and require read response frames to declare link healthy. */
        std::array<std::atomic<std::int64_t>, kMaxTools> tool_hb_start_ms_{};
        std::array<std::atomic<std::int64_t>, kMaxTools> tool_hb_tx_ms_{};
        std::array<std::atomic<std::int64_t>, kMaxTools> tool_hb_last_rx_ms_{};
        std::array<std::atomic<bool>, kMaxTools> tool_hb_offline_reported_{};
        std::array<std::atomic<std::int64_t>, kMaxTools> tool_sent_at_ms_{};  // 最近一次发送时刻（超时判定用）

        // --- write() 时间门控调度（工具发送并入主控制循环，无锁单发送方） ---
        // 机械臂主帧每次 write() 提交；工具通道由 43 版 SDK 独立发送。
        // 频率由参数显式配置（tool_ctrl_rate/tool_read_rate/tool_hb_rate），
        // 时间门控天然适配任何主循环频率（yaml update_rate 可为 100/250/500/1000Hz）。
        double tool_ctrl_rate_ = 10.0;   // 工具控制指令频率 Hz
        double tool_read_rate_ = 50.0;   // 工具读请求频率 Hz
        double tool_hb_rate_ = 0.1;      // 心跳频率 Hz
        std::chrono::steady_clock::duration tool_ctrl_interval_{};
        std::chrono::steady_clock::duration tool_read_interval_{};
        std::chrono::steady_clock::duration tool_hb_interval_{};
        std::chrono::steady_clock::time_point last_tool_ctrl_tx_{};
        std::chrono::steady_clock::time_point last_tool_read_tx_{};
        std::chrono::steady_clock::time_point last_tool_hb_tx_{};
        std::chrono::steady_clock::time_point last_rs485_rx_poll_{};
        static constexpr std::chrono::milliseconds kRs485RxInterval{10};  // 100 Hz
        static constexpr std::int64_t kToolReplyTimeoutMs = 200;  // 工具写/读响应超时窗口（发送后 200ms 未响应）

        // --- Async per-cycle request/response pairing & timeout accounting ---
        // Fully decoupled design: sender thread only marks a frame pending; hardware
        // read() clears the pending flag when it parses a reply (CM update rate).
        // At the START of the next control cycle, if pending is still set, count timeout.
        std::array<std::atomic<bool>, kMaxTools> tool_reply_pending_{};     // true = sent, awaiting reply
        std::array<std::atomic<int>, kMaxTools> tool_reply_kind_{};         // 0=none,1=read,2=write (for logging)
        std::array<std::atomic<std::uint64_t>, kMaxTools> tool_tx_total_{}; // total frames sent
        std::array<std::atomic<std::uint64_t>, kMaxTools> tool_rx_total_{}; // frames answered (pending cleared by recv)
        std::array<std::atomic<std::uint64_t>, kMaxTools> tool_timeout_count_{}; // frames never answered within cycle
        // Last reported snapshot of timeout counters (for periodic summary diffs).
        std::array<std::uint64_t, kMaxTools> tool_last_reported_timeout_{};
        std::array<std::chrono::steady_clock::time_point, kMaxTools> tool_last_summary_tp_{};

        // 工具通道通信调试日志开关（ros2 param tool_debug_log，默认关闭）。
        // 开启后打印 pending/read_gate/RX valid|invalid/rx_drop 等工具帧级日志，便于排查。
        std::atomic<bool> tool_debug_log_{false};

        void tool_callback_for_tool(size_t tool_idx);
        /** Async: send-only (getStatus/move); replies observed in hardware read(). */
        void tool_callback_for_tool_async(size_t tool_idx);
        /** Sole OnGetChData path: COM1 tools + COM2 KWR75, called from read(). */
        void pollRs485InHardwareRead();
        void dispatchToolCom1Frame(size_t tool_idx, const unsigned char* data, long received);
        void ingestKwr75Com2Frame(bool left_arm, const unsigned char* data, long received);
        /** Read once from channel, copy to data_buf, return byte count or 0. */
        long receiveToolResponse(unsigned char* data_buf, size_t buf_size, GetChDataFunc get_ch_data, long channel);
        /** Parse one complete tool frame; false means discard it. */
        bool processToolResponse(const unsigned char* data_buf, size_t size, size_t gripper_idx);
        bool isToolStateCloseToCommand(size_t tool_idx, double threshold);
        /** True if tool is stopped (hand: at command and stabilized; gripper: at target and stopped). */
        bool isToolStopped(size_t tool_idx);
        /**
         * 计数器调度：在 write() 中向 SDK 的独立工具通道提交一条指令。
         * 优先级：① 控制工具指令 > ② 获取状态指令(50Hz) > ③ 心跳。
         */
        void sendToolCommandOnce(size_t tool_idx, bool ctrl_due, bool read_due, bool hb_due);
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
