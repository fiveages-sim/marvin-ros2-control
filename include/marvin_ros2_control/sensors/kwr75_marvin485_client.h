#pragma once

#include <array>
#include <chrono>
#include <cstdint>

#include "marvin_ros2_control/sensors/kwr75_protocol.h"
#include "marvin_ros2_control/tool/modbus_io.h"

namespace marvin_ros2_control
{
    /**
     * KWR75 helper: send start/poll on COM2; RX frames are ingested by
     * MarvinHardware::pollRs485InHardwareRead() into sample_slot (seqlock).
     */
    class Kwr75Marvin485Client
    {
    public:
        Kwr75Marvin485Client(
            Send485Func send_485,
            long channel,
            Kwr75LockFreeSample* sample_slot,
            uint8_t command_code = 0x48,
            bool convert_to_si = true,
            double gravity = 9.80665,
            int warmup_timeout_ms = 500);

        bool warmup();
        /** Take a new sample from hardware-read seqlock; convert to SI wrench. */
        bool takeWrench(std::array<double, Kwr75Protocol::kAxisCount>& wrench_si);
        /** 0x49: send one poll, then wait until deadline for a sample. */
        bool pollAndTakeWrench(
            std::chrono::steady_clock::time_point deadline,
            std::array<double, Kwr75Protocol::kAxisCount>& wrench_si);

        bool isPollMode() const { return command_code_ == 0x49; }

    private:
        bool sendStartCommand();

        Send485Func send_485_;
        long channel_;
        Kwr75LockFreeSample* sample_slot_;
        uint8_t command_code_;
        bool convert_to_si_;
        double gravity_;
        int warmup_timeout_ms_;
        bool started_ = false;
    };
}  // namespace marvin_ros2_control
