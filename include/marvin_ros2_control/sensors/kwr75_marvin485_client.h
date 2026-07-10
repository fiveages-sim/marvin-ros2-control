#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "marvin_ros2_control/sensors/kwr75_protocol.h"
#include "marvin_ros2_control/tool/modbus_io.h"

namespace marvin_ros2_control
{
    using Kwr75RxLogFunc = std::function<void(long received, long rx_channel)>;

    /** KWR75 reader over Tianji/Marvin SDK RS485 (OnSetChData / OnGetChData). */
    class Kwr75Marvin485Client
    {
    public:
        Kwr75Marvin485Client(
            Send485Func send_485,
            GetChDataFunc get_ch_data,
            long channel,
            uint8_t command_code = 0x49,
            bool convert_to_si = true,
            double gravity = 9.80665,
            int response_timeout_ms = 50,
            int recv_interval_ms = 20,
            int warmup_timeout_ms = 500,
            Kwr75RxLogFunc rx_log = nullptr);

        ~Kwr75Marvin485Client();

        Kwr75Marvin485Client(const Kwr75Marvin485Client&) = delete;
        Kwr75Marvin485Client& operator=(const Kwr75Marvin485Client&) = delete;

        bool warmup();
        bool readWrench(std::array<double, Kwr75Protocol::kAxisCount>& wrench_si);
        /** 0x49: retry send→recv until deadline; 0x48: wait for new frame until deadline. */
        bool readWrenchUntil(
            std::chrono::steady_clock::time_point deadline,
            std::array<double, Kwr75Protocol::kAxisCount>& wrench_si);
        void stop();
        void stopRecvThread();

        /** Hex dump of recent OnGetChData bytes when warmup/read fails. */
        std::string lastIoSampleHex() const { return last_io_sample_hex_; }
        long lastRxChannel() const { return last_rx_channel_; }

    private:
        void recordIoSample(const std::vector<uint8_t>& buffer, long rx_channel);
        void logRxBytes(long received, long rx_channel) const;
        bool usesPollPerRead() const;
        bool usesStreaming() const;
        bool ensureStreaming();
        bool sendStartCommand();
        bool readPollResponseFrame(std::array<uint8_t, Kwr75Protocol::kFrameLength>& frame);
        /** 0x49: one send on COM2, then one 28-byte recv on COM2. */
        bool pollOnceTransaction(std::array<float, Kwr75Protocol::kAxisCount>* raw_out);
        void drainStaleCom2Rx();
        void startRecvThread();
        void recvThreadFunc();
        bool waitForFirstSample();
        bool takeNewRaw(std::array<float, Kwr75Protocol::kAxisCount>& raw_out);

        Send485Func send_485_;
        GetChDataFunc get_ch_data_;
        long channel_;
        uint8_t command_code_;
        bool convert_to_si_;
        double gravity_;
        int response_timeout_ms_;
        int recv_interval_ms_;
        int warmup_timeout_ms_;
        Kwr75RxLogFunc rx_log_;
        bool streaming_started_ = false;
        std::string last_io_sample_hex_;
        long last_rx_channel_ = -1;

        std::atomic<bool> recv_running_{false};
        std::thread recv_thread_;
        std::mutex sample_mutex_;
        std::array<float, Kwr75Protocol::kAxisCount> latest_raw_{};
        bool has_sample_ = false;
        bool has_pending_ = false;
    };
}  // namespace marvin_ros2_control
