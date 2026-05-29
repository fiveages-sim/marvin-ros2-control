#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace marvin_ros2_control
{
    class FreedomArmCanProtocol
    {
    public:
        static constexpr std::size_t kJointCount = 6;
        static constexpr uint8_t kMoveCommand = 0x10;
        static constexpr uint8_t kAngleQueryCommand = 0xF1;
        static constexpr uint8_t kMaxProtocolAngle = 90;

        static std::vector<std::vector<uint8_t>> buildMoveFrames(
            uint8_t device_id,
            const std::array<uint8_t, kJointCount>& angles)
        {
            std::vector<std::vector<uint8_t>> frames;
            frames.reserve(2);

            frames.push_back(withLittleEndianCanId(
                makeExtendedId(device_id, kMoveCommand, 2, 1),
                {0x01, angles[0], 0x01, angles[1], 0x01, angles[2], 0x01, angles[3]}));

            frames.push_back(withLittleEndianCanId(
                makeExtendedId(device_id, kMoveCommand, 2, 2),
                {0x01, angles[4], 0x01, angles[5]}));

            return frames;
        }

        static std::vector<uint8_t> buildAngleQueryFrame(uint8_t device_id)
        {
            return withLittleEndianCanId(
                makeExtendedId(device_id, kAngleQueryCommand, 1, 1),
                {0x00});
        }

        static bool parseFeedbackFrame(
            uint8_t expected_device_id,
            const uint8_t* data,
            std::size_t size,
            std::array<uint8_t, kJointCount>& angles)
        {
            if (data == nullptr || size < kCanIdBytes + kJointCount)
            {
                return false;
            }

            const uint32_t can_id = readLittleEndianCanId(data);
            if (frameDeviceId(can_id) != expected_device_id ||
                frameCommand(can_id) != kAngleQueryCommand)
            {
                return false;
            }

            for (std::size_t i = 0; i < kJointCount; ++i)
            {
                angles[i] = std::min<uint8_t>(data[kCanIdBytes + i], kMaxProtocolAngle);
            }
            return true;
        }

        static uint32_t makeExtendedId(
            uint8_t device_id,
            uint8_t command,
            uint8_t total_frames,
            uint8_t frame_seq)
        {
            return (static_cast<uint32_t>(device_id & 0x1F) << 24) |
                   (static_cast<uint32_t>(command) << 16) |
                   (static_cast<uint32_t>(total_frames) << 8) |
                   static_cast<uint32_t>(frame_seq);
        }

    private:
        static constexpr std::size_t kCanIdBytes = 4;

        static std::vector<uint8_t> withLittleEndianCanId(
            uint32_t can_id,
            std::initializer_list<uint8_t> payload)
        {
            std::vector<uint8_t> frame;
            frame.reserve(kCanIdBytes + payload.size());
            frame.push_back(static_cast<uint8_t>(can_id & 0xFF));
            frame.push_back(static_cast<uint8_t>((can_id >> 8) & 0xFF));
            frame.push_back(static_cast<uint8_t>((can_id >> 16) & 0xFF));
            frame.push_back(static_cast<uint8_t>((can_id >> 24) & 0xFF));
            frame.insert(frame.end(), payload.begin(), payload.end());
            return frame;
        }

        static uint32_t readLittleEndianCanId(const uint8_t* data)
        {
            return static_cast<uint32_t>(data[0]) |
                   (static_cast<uint32_t>(data[1]) << 8) |
                   (static_cast<uint32_t>(data[2]) << 16) |
                   (static_cast<uint32_t>(data[3]) << 24);
        }

        static uint8_t frameDeviceId(uint32_t can_id)
        {
            return static_cast<uint8_t>((can_id >> 24) & 0x1F);
        }

        static uint8_t frameCommand(uint32_t can_id)
        {
            return static_cast<uint8_t>((can_id >> 16) & 0xFF);
        }
    };
} // namespace marvin_ros2_control
