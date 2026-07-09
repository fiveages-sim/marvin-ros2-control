#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "marvin_ros2_control/sensors/kwr75_protocol.h"
#include "marvin_ros2_control/tool/modbus_io.h"

namespace marvin_ros2_control
{
    /** KWR75 reader over Tianji/Marvin SDK RS485 (OnSetChData / OnGetChData). */
    class Kwr75Marvin485Client
    {
    public:
        Kwr75Marvin485Client(
            Send485Func send_485,
            GetChDataFunc get_ch_data,
            long channel,
            uint8_t command_code = 0x48,
            bool convert_to_si = true,
            double gravity = 9.80665,
            int response_timeout_ms = 50);

        bool warmup();
        bool readWrench(std::array<double, Kwr75Protocol::kAxisCount>& wrench_si);
        /** Hex dump of recent OnGetChData bytes when warmup/read fails. */
        std::string lastIoSampleHex() const { return last_io_sample_hex_; }
        long lastRxChannel() const { return last_rx_channel_; }

    private:
        void recordIoSample(const std::vector<uint8_t>& buffer, long rx_channel);
        bool usesPollPerRead() const;
        bool ensureStreaming();
        bool sendStartCommand();
        bool readResponseFrame(std::array<uint8_t, Kwr75Protocol::kFrameLength>& frame);

        Send485Func send_485_;
        GetChDataFunc get_ch_data_;
        long channel_;
        uint8_t command_code_;
        bool convert_to_si_;
        double gravity_;
        int response_timeout_ms_;
        bool streaming_started_ = false;
        std::string last_io_sample_hex_;
        long last_rx_channel_ = -1;
    };
}  // namespace marvin_ros2_control
