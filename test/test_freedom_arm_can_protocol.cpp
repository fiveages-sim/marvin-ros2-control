#include "marvin_ros2_control/tool/hands/freedom/freedom_arm_can_protocol.h"

#include <gtest/gtest.h>

#include <array>

namespace
{

TEST(FreedomArmCanProtocol, BuildsMoveFramesWithLittleEndianExtendedId)
{
    const std::array<uint8_t, marvin_ros2_control::FreedomArmCanProtocol::kJointCount> angles{
        1, 2, 3, 4, 5, 6};

    const auto frames = marvin_ros2_control::FreedomArmCanProtocol::buildMoveFrames(3, angles);

    ASSERT_EQ(frames.size(), 2u);

    EXPECT_EQ(frames[0], (std::vector<uint8_t>{
        0x01, 0x02, 0x10, 0x03,
        0x01, 0x01, 0x01, 0x02, 0x01, 0x03, 0x01, 0x04}));
    EXPECT_EQ(frames[1], (std::vector<uint8_t>{
        0x02, 0x02, 0x10, 0x03,
        0x01, 0x05, 0x01, 0x06}));
}

TEST(FreedomArmCanProtocol, BuildsAngleQueryFrame)
{
    const auto frame = marvin_ros2_control::FreedomArmCanProtocol::buildAngleQueryFrame(1);

    EXPECT_EQ(frame, (std::vector<uint8_t>{
        0x01, 0x01, 0xF1, 0x01,
        0x00}));
}

TEST(FreedomArmCanProtocol, ParsesFeedbackFrame)
{
    const std::vector<uint8_t> frame{
        0x01, 0x01, 0xF1, 0x01,
        0, 45, 90, 10, 20, 30};

    std::array<uint8_t, marvin_ros2_control::FreedomArmCanProtocol::kJointCount> angles{};
    EXPECT_TRUE(marvin_ros2_control::FreedomArmCanProtocol::parseFeedbackFrame(1, frame.data(), frame.size(), angles));
    EXPECT_EQ(angles, (std::array<uint8_t, marvin_ros2_control::FreedomArmCanProtocol::kJointCount>{0, 45, 90, 10, 20, 30}));
}

}  // namespace
