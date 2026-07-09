#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <string>

namespace marvin_ros2_control
{
    /** Marvin internal RS485 COM2 (SDK channel 3). Tools default to COM1 (channel 2). */
    constexpr long KWR75_FT_CHANNEL = 3;

    /** KWR75 RS485 frame layout (28 bytes, default start 0x48 AA 0D 0A). */
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
                wire_bytes[3],
                wire_bytes[2],
                wire_bytes[1],
                wire_bytes[0],
            };
            float value = 0.0F;
            std::memcpy(&value, ieee754, sizeof(value));
            return value;
        }

        static bool parseFrame(
            const std::array<uint8_t, kFrameLength>& frame,
            std::array<float, kAxisCount>& values)
        {
            if ((frame[0] != 0x48 && frame[0] != 0x49) || frame[1] != kFrameMarker)
            {
                return false;
            }
            if (frame[kFrameLength - 2] != kFrameEnd0 || frame[kFrameLength - 1] != kFrameEnd1)
            {
                return false;
            }

            for (std::size_t axis = 0; axis < kAxisCount; ++axis)
            {
                const std::size_t offset = kPayloadOffset + axis * kFloatBytes;
                values[axis] = decodeWireFloat(frame.data() + offset);
            }
            return true;
        }

        static bool findFrameInBuffer(
            const uint8_t* data,
            std::size_t data_size,
            uint8_t command_code,
            std::array<uint8_t, kFrameLength>& frame_out)
        {
            if (data_size < kFrameLength)
            {
                return false;
            }

            for (std::size_t start = 0; start + kFrameLength <= data_size; ++start)
            {
                if (data[start] != command_code && data[start] != 0x48)
                {
                    continue;
                }
                if (data[start + 1] != kFrameMarker)
                {
                    continue;
                }
                if (data[start + kFrameLength - 2] != kFrameEnd0 ||
                    data[start + kFrameLength - 1] != kFrameEnd1)
                {
                    continue;
                }
                std::copy_n(data + static_cast<std::ptrdiff_t>(start), kFrameLength, frame_out.begin());
                return true;
            }
            return false;
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
        long left_channel = KWR75_FT_CHANNEL;
        long right_channel = KWR75_FT_CHANNEL;
        int poll_interval_ms = 10;
        uint8_t command_code = 0x48;
        bool convert_to_si = true;
        double gravity = 9.80665;
        int response_timeout_ms = 50;
        std::string left_frame_id = "left_eef";
        std::string right_frame_id = "right_eef";
        std::string left_wrench_topic = "/left_arm_external_wrench";
        std::string right_wrench_topic = "/right_arm_external_wrench";

        static Kwr75FtConfig defaults() { return Kwr75FtConfig{}; }
    };
}  // namespace marvin_ros2_control
