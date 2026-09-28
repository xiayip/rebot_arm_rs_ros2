#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <thread>

#include "rebot_robstride_protocol/rebot_gripper_protocol.hpp"

namespace
{
namespace codec = rebot_robstride_protocol::codec;

shared_can_interface::v1::DeviceConfiguration gripper_configuration()
{
  shared_can_interface::v1::DeviceConfiguration device;
  device.name = "gripper_command_joint";
  device.command_interfaces = {"position", "velocity"};
  device.state_interfaces = {"position", "velocity", "effort", "temperature"};
  device.parameters = {
    {"motor_id", "7"}, {"model", "rs-00"}, {"direction", "1"},
    {"offset", "0"}, {"mit_kp", "50"}, {"mit_kd", "4"},
    {"open_position", "5"}, {"closed_position", "0"},
    {"default_velocity", "0.25"}};
  return device;
}

codec::Frame parameter_reply(std::uint16_t index, float value)
{
  codec::Frame frame;
  frame.arbitration_id = codec::build_extended_id(
    codec::CommunicationType::kReadParameter, 7, 0xFD);
  frame.data[0] = static_cast<std::uint8_t>(index & 0xFFU);
  frame.data[1] = static_cast<std::uint8_t>(index >> 8U);
  std::memcpy(frame.data.data() + 4, &value, sizeof(value));
  return frame;
}

codec::Frame status(float position, float velocity = 0.0F)
{
  auto frame = codec::make_mit_command(
    7, codec::limits_for_model("rs-00"), position, velocity, 0.0F, 0.0F, 0.0F);
  frame.arbitration_id = codec::build_extended_id(
    codec::CommunicationType::kOperationStatus, 7, 0xFD);
  return frame;
}

float command_position(const codec::Frame & command)
{
  auto frame = command;
  frame.arbitration_id = codec::build_extended_id(
    codec::CommunicationType::kOperationStatus, 7, 0xFD);
  const auto decoded = codec::decode_status(frame, codec::limits_for_model("rs-00"));
  EXPECT_TRUE(decoded.has_value());
  return decoded ? decoded->position : 0.0F;
}

}  // namespace

TEST(RebotGripperProtocol, MapsMotorEndpointsToNormalizedState)
{
  rebot_robstride_protocol::RebotGripperProtocol protocol;
  protocol.configure({{"host_id", "0xFD"}}, {gripper_configuration()});

  ASSERT_TRUE(protocol.process_frame(status(5.0F), std::chrono::steady_clock::now()));
  double value = -1.0;
  ASSERT_TRUE(protocol.get_state("gripper_command_joint", "position", value));
  EXPECT_NEAR(value, 0.0, 1e-4);

  ASSERT_TRUE(protocol.process_frame(status(2.5F), std::chrono::steady_clock::now()));
  ASSERT_TRUE(protocol.get_state("gripper_command_joint", "position", value));
  EXPECT_NEAR(value, 0.5, 1e-4);

  ASSERT_TRUE(protocol.process_frame(status(0.0F), std::chrono::steady_clock::now()));
  ASSERT_TRUE(protocol.get_state("gripper_command_joint", "position", value));
  EXPECT_NEAR(value, 1.0, 1e-4);
}

TEST(RebotGripperProtocol, MapsInitialOpenMotorPositionToOpenLogicalCommand)
{
  rebot_robstride_protocol::RebotGripperProtocol protocol;
  protocol.configure({{"host_id", "0xFD"}}, {gripper_configuration()});
  const auto now = std::chrono::steady_clock::now();
  ASSERT_TRUE(protocol.process_frame(parameter_reply(0x7019, 5.0F), now));
  ASSERT_TRUE(protocol.process_frame(parameter_reply(0x7005, 0.0F), now));
  ASSERT_TRUE(protocol.process_frame(parameter_reply(0x701A, 0.0F), now));
  ASSERT_TRUE(protocol.initial_state_ready());

  double command = -1.0;
  ASSERT_TRUE(protocol.get_initial_command("gripper_command_joint", "position", command));
  EXPECT_NEAR(command, 0.0, 1e-9);
  ASSERT_TRUE(protocol.get_initial_command("gripper_command_joint", "velocity", command));
  EXPECT_NEAR(command, 0.25, 1e-9);
}

TEST(RebotGripperProtocol, SlewLimitsNormalizedTargetBeforeMitEncoding)
{
  rebot_robstride_protocol::RebotGripperProtocol protocol;
  protocol.configure({{"host_id", "0xFD"}}, {gripper_configuration()});
  ASSERT_TRUE(protocol.process_frame(status(5.0F), std::chrono::steady_clock::now()));
  ASSERT_TRUE(protocol.set_command("gripper_command_joint", "velocity", 0.5));
  ASSERT_TRUE(protocol.set_command("gripper_command_joint", "position", 1.0));

  std::vector<shared_can_interface::v1::CanFrame> commands;
  protocol.command_frames(commands);
  ASSERT_EQ(commands.size(), 1U);
  EXPECT_NEAR(command_position(commands.front()), 5.0, 4e-4);

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  protocol.command_frames(commands);
  ASSERT_EQ(commands.size(), 1U);
  const double motor_target = command_position(commands.front());
  EXPECT_LT(motor_target, 5.0);
  EXPECT_GT(motor_target, 4.85);
}

TEST(RebotGripperProtocol, RejectsOutOfRangeAndUnsupportedCommands)
{
  rebot_robstride_protocol::RebotGripperProtocol protocol;
  protocol.configure({{"host_id", "0xFD"}}, {gripper_configuration()});
  EXPECT_FALSE(protocol.set_command("gripper_command_joint", "position", -0.1));
  EXPECT_FALSE(protocol.set_command("gripper_command_joint", "position", 1.1));
  EXPECT_FALSE(protocol.set_command("gripper_command_joint", "velocity", 1.1));
  EXPECT_FALSE(protocol.set_command("gripper_command_joint", "effort", 0.5));
}
