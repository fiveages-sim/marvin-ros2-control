#pragma once

#include "marvin_ros2_control/grippers/gripper_control.h"

namespace marvin_ros2_control
{
    /**
     * @brief Dexterous Hand implementation for Marvin robots
     * 
     * Controls a multi-fingered dexterous hand with 7 joints:
     * - Thumb: Pitch, Yaw, Roll
     * - Index, Middle, Ring, Little: Pitch only
     * 
     * Each joint has position, torque, speed, and lock rotor protection parameters.
     */
    class DexterousHandGripper : public ModbusHand
    {
    public:
        DexterousHandGripper(Clear485Func clear_485, Send485Func send_485,
                             GetChDataFunc on_get_ch_data = nullptr);
        bool initialize() override;
        bool move_gripper(int torque, int velocity, double position) override;
        bool getStatus() override;
        void deinitialize() override;
        void resetState() override;

        // 7-DOF control method (7 joints: Thumb_Pitch, Thumb_Yaw, Index_Pitch, Middle_Pitch, Ring_Pitch, Little_Pitch, Thumb_Roll)
        bool move_gripper(const std::vector<int>& torques, const std::vector<int>& velocities, const std::vector<double>& positions);

        // Process received Modbus response for 7 joints
        // Returns true if response was successfully processed
        bool processReadResponse(const uint8_t* data, size_t data_size, 
                                int& torque, int& velocity, double& position) override;

        // Update status from parsed Modbus response
        void updateStatusFromResponse(const std::vector<uint16_t>& registers) override;

        // Get joint positions (normalized 0.0-1.0)
        const std::array<double, 7>& getJointPositions() const { return joint_positions_; }
        const std::array<double, 7>& getJointVelocities() const { return joint_velocities_; }
        const std::array<double, 7>& getJointEfforts() const { return joint_efforts_; }

        // Additional methods for individual finger control
        bool setFingerPosition(uint8_t finger_id, uint8_t position);
        bool setFingerTorque(uint8_t finger_id, uint8_t torque);
        bool setFingerSpeed(uint8_t finger_id, uint8_t speed);
        bool getFingerPosition(uint8_t finger_id, uint8_t& position);
        bool getFingerTorque(uint8_t finger_id, uint8_t& torque);
        bool getFingerSpeed(uint8_t finger_id, uint8_t& speed);

    private:
        // Modbus configuration
        static constexpr uint8_t SLAVE_ID = 0x01;  // Default slave address (can be configured)
        static constexpr uint8_t WRITE_SINGLE_FUNCTION = 0x06;  // Write Single Register
        static constexpr uint8_t READ_FUNCTION = 0x03;         // Read Holding Registers

        // Register addresses (Holding Registers 0-42)
        // Position registers (0-6)
        static constexpr uint16_t THUMB_PITCH_REG = 0x0000;
        static constexpr uint16_t THUMB_YAW_REG = 0x0001;
        static constexpr uint16_t INDEX_PITCH_REG = 0x0002;
        static constexpr uint16_t MIDDLE_PITCH_REG = 0x0003;
        static constexpr uint16_t RING_PITCH_REG = 0x0004;
        static constexpr uint16_t LITTLE_PITCH_REG = 0x0005;
        static constexpr uint16_t THUMB_ROLL_REG = 0x0006;

        // Torque registers (7-13)
        static constexpr uint16_t THUMB_PITCH_TORQUE_REG = 0x0007;
        static constexpr uint16_t THUMB_YAW_TORQUE_REG = 0x0008;
        static constexpr uint16_t INDEX_PITCH_TORQUE_REG = 0x0009;
        static constexpr uint16_t MIDDLE_PITCH_TORQUE_REG = 0x000A;
        static constexpr uint16_t RING_PITCH_TORQUE_REG = 0x000B;
        static constexpr uint16_t LITTLE_PITCH_TORQUE_REG = 0x000C;
        static constexpr uint16_t THUMB_ROLL_TORQUE_REG = 0x000D;

        // Speed registers (14-20)
        static constexpr uint16_t THUMB_PITCH_SPEED_REG = 0x000E;
        static constexpr uint16_t THUMB_YAW_SPEED_REG = 0x000F;
        static constexpr uint16_t INDEX_PITCH_SPEED_REG = 0x0010;
        static constexpr uint16_t MIDDLE_PITCH_SPEED_REG = 0x0011;
        static constexpr uint16_t RING_PITCH_SPEED_REG = 0x0012;
        static constexpr uint16_t LITTLE_PITCH_SPEED_REG = 0x0013;
        static constexpr uint16_t THUMB_ROLL_SPEED_REG = 0x0014;

        // Lock rotor threshold registers (21-27)
        static constexpr uint16_t THUMB_PITCH_LOCK_THRESHOLD_REG = 0x0015;
        static constexpr uint16_t THUMB_YAW_LOCK_THRESHOLD_REG = 0x0016;
        static constexpr uint16_t INDEX_PITCH_LOCK_THRESHOLD_REG = 0x0017;
        static constexpr uint16_t MIDDLE_PITCH_LOCK_THRESHOLD_REG = 0x0018;
        static constexpr uint16_t RING_PITCH_LOCK_THRESHOLD_REG = 0x0019;
        static constexpr uint16_t LITTLE_PITCH_LOCK_THRESHOLD_REG = 0x001A;
        static constexpr uint16_t THUMB_ROLL_LOCK_THRESHOLD_REG = 0x001B;

        // Lock rotor time registers (28-34)
        static constexpr uint16_t THUMB_PITCH_LOCK_TIME_REG = 0x001C;
        static constexpr uint16_t THUMB_YAW_LOCK_TIME_REG = 0x001D;
        static constexpr uint16_t INDEX_PITCH_LOCK_TIME_REG = 0x001E;
        static constexpr uint16_t MIDDLE_PITCH_LOCK_TIME_REG = 0x001F;
        static constexpr uint16_t RING_PITCH_LOCK_TIME_REG = 0x0020;
        static constexpr uint16_t LITTLE_PITCH_LOCK_TIME_REG = 0x0021;
        static constexpr uint16_t THUMB_ROLL_LOCK_TIME_REG = 0x0022;

        // Lock rotor torque registers (35-41)
        static constexpr uint16_t THUMB_PITCH_LOCK_TORQUE_REG = 0x0023;
        static constexpr uint16_t THUMB_YAW_LOCK_TORQUE_REG = 0x0024;
        static constexpr uint16_t INDEX_PITCH_LOCK_TORQUE_REG = 0x0025;
        static constexpr uint16_t MIDDLE_PITCH_LOCK_TORQUE_REG = 0x0026;
        static constexpr uint16_t RING_PITCH_LOCK_TORQUE_REG = 0x0027;
        static constexpr uint16_t LITTLE_PITCH_LOCK_TORQUE_REG = 0x0028;
        static constexpr uint16_t THUMB_ROLL_LOCK_TORQUE_REG = 0x0029;

        // Pressure sensing data selection (42)
        static constexpr uint16_t PRESSURE_SENSING_DATA_SELECTION_REG = 0x002A;

        // Helper methods
        uint16_t getPositionRegister(uint8_t finger_id) const;
        uint16_t getTorqueRegister(uint8_t finger_id) const;
        uint16_t getSpeedRegister(uint8_t finger_id) const;

        // Joint state (7 joints: normalized 0.0-1.0)
        std::array<double, 7> joint_positions_ = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
        std::array<double, 7> joint_velocities_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::array<double, 7> joint_efforts_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        bool status_valid_ = false;
    };
} // namespace marvin_ros2_control

