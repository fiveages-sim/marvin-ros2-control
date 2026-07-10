#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

#include "marvin_ros2_control/tool/modbus_io.h"

namespace marvin_ros2_control
{
    /** Marvin SDK COM2 (KWR75 FT). Gripper uses COM1. Independent COM buses. */
    constexpr long KWR75_FT_CHANNEL = COM2_CHANNEL;

    /**
     * KWR75 RS485 frame (28 bytes fixed length).
     *
     * Layout: [cmd][0xAA][Fx][Fy][Fz][Mx][My][Mz][0x0D][0x0A]
     * - cmd: 0x48 or 0x49 (matches host start command)
     * - each component: 4-byte IEEE754 float, low byte first on wire
     *   (wire B0 75 1C C1 -> float bytes 0xC11C75B0)
     * - units: Fx..Fz (kg), Mx..Mz (kg·m)
     *
     * Parse only when OnGetChData returns exactly 28 bytes matching this layout.
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

        static bool isValidCommandByte(uint8_t byte)
        {
            return byte == 0x48 || byte == 0x49;
        }

        static std::array<uint8_t, 4> buildPollRequest(uint8_t command_code = 0x48)
        {
            return {command_code, kFrameMarker, kFrameEnd0, kFrameEnd1};
        }

        /** Wire float: 4 bytes low-byte-first -> host float (e.g. B0751CC1 -> 0xC11C75B0). */
        static float decodeWireFloat(const uint8_t* wire_bytes)
        {
            const uint8_t ieee754[4] = {
                wire_bytes[3],
                wire_bytes[2],
                wire_bytes[1],
                wire_bytes[0],
            };
            float value = 0.0F;
            std::memcpy(&value, ieee754, sizeof(value));
            return value;
        }

        /** True only for a full 28-byte frame with valid header/trailer (no partial buffer parse). */
        static bool isValidCompleteFrame(const uint8_t* data, std::size_t data_size)
        {
            if (data == nullptr || data_size != kFrameLength)
            {
                return false;
            }
            if (!isValidCommandByte(data[0]))
            {
                return false;
            }
            if (data[1] != kFrameMarker)
            {
                return false;
            }
            if (data[kFrameLength - 2] != kFrameEnd0 || data[kFrameLength - 1] != kFrameEnd1)
            {
                return false;
            }
            return true;
        }

        static bool frameHeaderMatches(const uint8_t* data, uint8_t command_code)
        {
            if (!isValidCompleteFrame(data, kFrameLength))
            {
                return false;
            }
            (void)command_code;
            return true;
        }

        /** Reject mis-synced frames (random bytes parsed as IEEE754 → e+20+). */
        static bool isPlausibleRawSample(const std::array<float, kAxisCount>& values)
        {
            for (std::size_t i = 0; i < kAxisCount; ++i)
            {
                const float v = values[i];
                if (!std::isfinite(v))
                {
                    return false;
                }
                const float limit = (i < 3) ? 2.0e3f : 2.0e2f;
                if (std::fabs(v) > limit)
                {
                    return false;
                }
            }
            return true;
        }

        static bool parseFrame(
            const std::array<uint8_t, kFrameLength>& frame,
            std::array<float, kAxisCount>& values)
        {
            if (!isValidCompleteFrame(frame.data(), frame.size()))
            {
                return false;
            }

            for (std::size_t axis = 0; axis < kAxisCount; ++axis)
            {
                const std::size_t offset = kPayloadOffset + axis * kFloatBytes;
                values[axis] = decodeWireFloat(frame.data() + offset);
            }
            return isPlausibleRawSample(values);
        }

        static bool findFrameInBuffer(
            const uint8_t* data,
            std::size_t data_size,
            uint8_t command_code,
            std::array<uint8_t, kFrameLength>& frame_out)
        {
            if (data_size != kFrameLength)
            {
                (void)command_code;
                return false;
            }
            if (!isValidCompleteFrame(data, data_size))
            {
                return false;
            }
            std::copy_n(data, kFrameLength, frame_out.begin());
            return true;
        }

        static bool findLastFrameInBuffer(
            const uint8_t* data,
            std::size_t data_size,
            uint8_t command_code,
            std::array<uint8_t, kFrameLength>& frame_out)
        {
            return findFrameInBuffer(data, data_size, command_code, frame_out);
        }

        /** Strict: buffer must be exactly one 28-byte frame (no scan/stitch). */
        static bool tryParseRxBuffer(
            const uint8_t* data,
            std::size_t data_size,
            uint8_t command_code,
            std::array<float, kAxisCount>& values_out)
        {
            return tryParseCompleteFrame(data, data_size, command_code, values_out);
        }

        /** OnGetChData must return exactly kFrameLength bytes; otherwise discard. */
        static bool isExactFrameRead(long received, long rx_channel, long want_channel)
        {
            return received == static_cast<long>(kFrameLength) && rx_channel == want_channel;
        }

        /** Parse one complete OnGetChData chunk (size == 28, valid 0x48/0x49 AA ... 0D 0A). */
        static bool tryParseCompleteFrame(
            const uint8_t* data,
            std::size_t data_size,
            uint8_t command_code,
            std::array<float, kAxisCount>& values_out)
        {
            (void)command_code;
            if (!isValidCompleteFrame(data, data_size))
            {
                return false;
            }
            std::array<uint8_t, kFrameLength> frame{};
            std::copy_n(data, kFrameLength, frame.begin());
            return parseFrame(frame, values_out);
        }

        static void rawToSi(
            const std::array<float, kAxisCount>& raw,
            std::array<double, kAxisCount>& wrench_si,
            bool convert_to_si,
            double gravity)
        {
            const double force_scale = convert_to_si ? gravity : 1.0;
            const double torque_scale = convert_to_si ? gravity : 1.0;
            for (std::size_t i = 0; i < kAxisCount; ++i)
            {
                const double scale = (i < 3) ? force_scale : torque_scale;
                wrench_si[i] = static_cast<double>(raw[i]) * scale;
            }
        }
    };

    /** KWR75 485-mode runtime parameters (defaults in code / xacro, not MarvinHardware). */
    struct Kwr75FtConfig
    {
        bool enabled = false;
        bool left_enabled = true;
        bool right_enabled = false;
        long left_channel = KWR75_FT_CHANNEL;
        long right_channel = KWR75_FT_CHANNEL;
        int poll_interval_ms = 100;
        /** COM2 recv thread cadence for 0x48 streaming (OnGetChData poll, no SDK lock). */
        int recv_interval_ms = 20;
        /** 0x48 = stream (one start cmd, recv thread only, saves RS485 bandwidth); 0x49 = poll per read. */
        uint8_t command_code = 0x48;
        bool convert_to_si = true;
        double gravity = 9.80665;
        int response_timeout_ms = 90;
        int warmup_timeout_ms = 500;
        std::string left_frame_id = "left_eef";
        std::string right_frame_id = "right_eef";
        std::string left_wrench_topic = "/left_arm_external_wrench";
        std::string right_wrench_topic = "/right_arm_external_wrench";

        static Kwr75FtConfig defaults() { return Kwr75FtConfig{}; }
    };
}  // namespace marvin_ros2_control
