#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <thread>

#include "marvin_ros2_control/tool/modbus_io.h"

namespace marvin_ros2_control
{
    constexpr long KWR75_FT_CHANNEL = COM2_CHANNEL;

    /**
     * KWR75 28-byte frame: [cmd][0xAA][Fx..Mz floats LE][0x0D][0x0A]
     * Only exact 28-byte chunks are accepted.
     */
    struct Kwr75Protocol
    {
        static constexpr std::size_t kAxisCount = 6;
        static constexpr std::size_t kFrameLength = 28;
        static constexpr uint8_t kFrameMarker = 0xAA;
        static constexpr uint8_t kFrameEnd0 = 0x0D;
        static constexpr uint8_t kFrameEnd1 = 0x0A;
        static constexpr std::size_t kPayloadOffset = 2;
        static constexpr std::size_t kFloatBytes = 4;

        static std::array<uint8_t, 4> buildPollRequest(uint8_t command_code = 0x48)
        {
            return {command_code, kFrameMarker, kFrameEnd0, kFrameEnd1};
        }

        static float decodeWireFloat(const uint8_t* wire_bytes)
        {
            const uint8_t ieee754[4] = {
                wire_bytes[3], wire_bytes[2], wire_bytes[1], wire_bytes[0]};
            float value = 0.0F;
            std::memcpy(&value, ieee754, sizeof(value));
            return value;
        }

        static bool isExactFrameRead(long received, long rx_channel, long want_channel)
        {
            return received == static_cast<long>(kFrameLength) && rx_channel == want_channel;
        }

        static bool tryParseCompleteFrame(
            const uint8_t* data,
            std::size_t data_size,
            uint8_t /*command_code*/,
            std::array<float, kAxisCount>& values_out)
        {
            if (data == nullptr || data_size != kFrameLength)
            {
                return false;
            }
            if ((data[0] != 0x48 && data[0] != 0x49) || data[1] != kFrameMarker)
            {
                return false;
            }
            if (data[kFrameLength - 2] != kFrameEnd0 || data[kFrameLength - 1] != kFrameEnd1)
            {
                return false;
            }
            for (std::size_t axis = 0; axis < kAxisCount; ++axis)
            {
                values_out[axis] = decodeWireFloat(data + kPayloadOffset + axis * kFloatBytes);
                const float v = values_out[axis];
                if (!std::isfinite(v) || std::fabs(v) > ((axis < 3) ? 2.0e3f : 2.0e2f))
                {
                    return false;
                }
            }
            return true;
        }

        static void rawToSi(
            const std::array<float, kAxisCount>& raw,
            std::array<double, kAxisCount>& wrench_si,
            bool convert_to_si,
            double gravity)
        {
            const double scale = convert_to_si ? gravity : 1.0;
            for (std::size_t i = 0; i < kAxisCount; ++i)
            {
                wrench_si[i] = static_cast<double>(raw[i]) * scale;
            }
        }
    };

    /** Lock-free seqlock: writer = hardware read COM2; reader = KWR75 pub thread. */
    struct Kwr75LockFreeSample
    {
        alignas(64) std::atomic<uint64_t> sequence{0};
        std::array<float, Kwr75Protocol::kAxisCount> raw{};
        std::atomic<bool> pending{false};
        std::atomic<bool> has_any{false};

        void publish(const std::array<float, Kwr75Protocol::kAxisCount>& values)
        {
            const uint64_t s0 = sequence.load(std::memory_order_relaxed);
            sequence.store(s0 + 1, std::memory_order_release);
            raw = values;
            sequence.store(s0 + 2, std::memory_order_release);
            pending.store(true, std::memory_order_release);
            has_any.store(true, std::memory_order_release);
        }

        bool tryConsume(std::array<float, Kwr75Protocol::kAxisCount>& out)
        {
            if (!pending.load(std::memory_order_acquire))
            {
                return false;
            }
            for (int spin = 0; spin < 8; ++spin)
            {
                const uint64_t s1 = sequence.load(std::memory_order_acquire);
                if (s1 & 1ULL)
                {
                    continue;
                }
                const auto copy = raw;
                const uint64_t s2 = sequence.load(std::memory_order_acquire);
                if (s1 == s2 && (s1 & 1ULL) == 0ULL)
                {
                    out = copy;
                    pending.store(false, std::memory_order_release);
                    return true;
                }
            }
            return false;
        }

        bool waitHasAny(int timeout_ms) const
        {
            const auto deadline = std::chrono::steady_clock::now() +
                                std::chrono::milliseconds(timeout_ms);
            while (std::chrono::steady_clock::now() <= deadline)
            {
                if (has_any.load(std::memory_order_acquire))
                {
                    return true;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            return false;
        }
    };

    struct Kwr75FtConfig
    {
        bool enabled = false;
        bool left_enabled = true;
        bool right_enabled = false;
        long left_channel = KWR75_FT_CHANNEL;
        long right_channel = KWR75_FT_CHANNEL;
        /** 0 = publish in hardware read() at ros2_control rate; >0 = dedicated poll thread. */
        int poll_interval_ms = 0;
        uint8_t command_code = 0x48;
        bool convert_to_si = true;
        double gravity = 9.80665;
        int warmup_timeout_ms = 500;
        std::string left_frame_id = "left_eef";
        std::string right_frame_id = "right_eef";
        std::string left_wrench_topic = "/left_arm_external_wrench";
        std::string right_wrench_topic = "/right_arm_external_wrench";

        static Kwr75FtConfig defaults() { return Kwr75FtConfig{}; }
    };
}  // namespace marvin_ros2_control
