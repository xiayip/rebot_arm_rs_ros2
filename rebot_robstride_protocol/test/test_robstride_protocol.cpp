#include <gtest/gtest.h>

#include <array>
#include <chrono>
#include <cmath>

#include "rebot_robstride_protocol/robstride_codec.hpp"
#include "rebot_robstride_protocol/robstride_protocol.hpp"

namespace codec = rebot_robstride_protocol::codec;

TEST(RobStrideCodec, ExtendedIdRoundTrip) {
  const auto encoded = codec::build_extended_id(
      codec::CommunicationType::kReadParameter, 0xABCD, 0x7F);
  const auto decoded = codec::parse_extended_id(encoded);
  EXPECT_EQ(decoded.communication_type,
            codec::CommunicationType::kReadParameter);
  EXPECT_EQ(decoded.extra_data, 0xABCD);
  EXPECT_EQ(decoded.node_id, 0x7F);
}

TEST(RobStrideCodec, ParameterReadUsesLittleEndianIndex) {
  const auto frame = codec::make_read_parameter(0x03, 0xFD, 0x7019);
  EXPECT_EQ(frame.arbitration_id, 0x1100FD03U);
  EXPECT_EQ(frame.data[0], 0x19);
  EXPECT_EQ(frame.data[1], 0x70);
}

TEST(RobStrideCodec, FloatWriteUsesLittleEndianValue) {
  const auto frame = codec::make_write_parameter_f32(0x04, 0xFD, 0x7016, 1.25F);
  EXPECT_EQ(frame.arbitration_id, 0x1200FD04U);
  EXPECT_EQ(frame.data[0], 0x16);
  EXPECT_EQ(frame.data[1], 0x70);
  EXPECT_EQ(frame.data[4], 0x00);
  EXPECT_EQ(frame.data[5], 0x00);
  EXPECT_EQ(frame.data[6], 0xA0);
  EXPECT_EQ(frame.data[7], 0x3F);
}

TEST(RobStrideCodec, MitCommandSeparatesTorqueIntoExtendedId) {
  const auto limits = codec::limits_for_model("rs-00");
  const auto base =
      codec::make_mit_command(6, limits, 0.1F, 0.2F, 3.0F, 0.4F, 0.5F);
  const auto torque_changed =
      codec::make_mit_command(6, limits, 0.1F, 0.2F, 3.0F, 0.4F, 0.8F);
  EXPECT_EQ(base.data, torque_changed.data);
  EXPECT_NE(codec::parse_extended_id(base.arbitration_id).extra_data,
            codec::parse_extended_id(torque_changed.arbitration_id).extra_data);
}

TEST(RobStrideCodec, StatusDecodeUsesModelLimitsAndFlags) {
  codec::Frame frame;
  frame.arbitration_id = codec::build_extended_id(
      codec::CommunicationType::kOperationStatus,
      static_cast<std::uint16_t>((2U << 14U) | (1U << 13U) | (1U << 9U) |
                                 0x03U),
      0xFD);
  frame.data = {0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x01, 0x2C};
  const auto status =
      codec::decode_status(frame, codec::limits_for_model("rs-06"));
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->device_id, 3);
  EXPECT_EQ(status->mode_state, 2);
  EXPECT_TRUE(status->uncalibrated);
  EXPECT_TRUE(status->overcurrent);
  EXPECT_NEAR(status->temperature_c, 30.0F, 1e-5F);
}

TEST(RobStrideCodec, ParameterResponseDecodesMotorAndFloat) {
  codec::Frame frame;
  frame.arbitration_id = codec::build_extended_id(
      codec::CommunicationType::kReadParameter, 0x0005, 0xFD);
  frame.data = {0x19, 0x70, 0, 0, 0, 0, 0x20, 0x40};
  const auto response = codec::decode_parameter_response(frame);
  ASSERT_TRUE(response.has_value());
  EXPECT_EQ(response->device_id, 5);
  EXPECT_EQ(response->parameter_id, 0x7019);
  EXPECT_FLOAT_EQ(codec::parameter_as_f32(*response), 2.5F);
}

TEST(RobStridePlugin, RejectsFramesForAnotherHostAndTracksInitialState) {
  rebot_robstride_protocol::RobStrideProtocol protocol;
  shared_can_interface::v1::ParameterMap bus{{"host_id", "0xFD"}};
  shared_can_interface::v1::DeviceConfiguration device;
  device.name = "joint1";
  device.command_interfaces = {"position"};
  device.state_interfaces = {"position", "velocity", "effort"};
  device.parameters = {{"motor_id", "1"},  {"model", "rs-06"},
                       {"direction", "1"}, {"offset", "0"},
                       {"mit_kp", "50"},   {"mit_kd", "3"}};
  protocol.configure(bus, {device});

  codec::Frame wrong_host;
  wrong_host.arbitration_id = codec::build_extended_id(
      codec::CommunicationType::kReadParameter, 1, 0xFE);
  wrong_host.data = {0x19, 0x70, 0, 0, 0, 0, 0, 0};
  EXPECT_FALSE(
      protocol.process_frame(wrong_host, std::chrono::steady_clock::now()));
  EXPECT_FALSE(protocol.initial_state_ready());

  auto reply = wrong_host;
  reply.arbitration_id = codec::build_extended_id(
      codec::CommunicationType::kReadParameter, 1, 0xFD);
  EXPECT_TRUE(protocol.process_frame(reply, std::chrono::steady_clock::now()));
  EXPECT_TRUE(protocol.initial_state_ready());
  double position = 1.0;
  EXPECT_TRUE(protocol.get_state("joint1", "position", position));
  EXPECT_DOUBLE_EQ(position, 0.0);
  EXPECT_TRUE(protocol.set_command("joint1", "position", 0.25));
  EXPECT_FALSE(protocol.set_command("joint1", "current", 0.25));
  std::vector<shared_can_interface::v1::CanFrame> commands;
  protocol.command_frames(commands);
  EXPECT_EQ(commands.size(), 1U);
  EXPECT_EQ(protocol.command_frame_capacity(), 1U);
}

TEST(RobStridePlugin, RejectsUnsupportedNamedInterfaces) {
  rebot_robstride_protocol::RobStrideProtocol protocol;
  shared_can_interface::v1::DeviceConfiguration device;
  device.name = "joint1";
  device.command_interfaces = {"current"};
  device.state_interfaces = {"position"};
  device.parameters = {{"motor_id", "1"},  {"model", "rs-06"},
                       {"direction", "1"}, {"offset", "0"},
                       {"mit_kp", "50"},   {"mit_kd", "3"}};
  EXPECT_THROW(protocol.configure({{"host_id", "0xFD"}}, {device}),
               std::invalid_argument);
}

TEST(RobStridePlugin, DerivesVelocityOnlyFromMechanicalPositionSamples) {
  rebot_robstride_protocol::RobStrideProtocol protocol;
  shared_can_interface::v1::DeviceConfiguration device;
  device.name = "joint1";
  device.command_interfaces = {"position"};
  device.state_interfaces = {"position", "velocity"};
  device.parameters = {{"motor_id", "1"},  {"model", "rs-06"},
                       {"direction", "1"}, {"offset", "0"},
                       {"mit_kp", "50"},   {"mit_kd", "3"}};
  protocol.configure({{"host_id", "0xFD"}}, {device});

  const auto start = std::chrono::steady_clock::now();
  codec::Frame position_reply;
  position_reply.arbitration_id = codec::build_extended_id(
      codec::CommunicationType::kReadParameter, 1, 0xFD);
  position_reply.data = {0x19, 0x70, 0, 0, 0, 0, 0, 0};
  ASSERT_TRUE(protocol.process_frame(position_reply, start));

  codec::Frame status_frame;
  status_frame.arbitration_id = codec::build_extended_id(
      codec::CommunicationType::kOperationStatus, 1, 0xFD);
  status_frame.data = {0x7F, 0xFF, 0x90, 0x00, 0x7F, 0xFF, 0x00, 0xFA};
  const auto decoded_status =
      codec::decode_status(status_frame, codec::limits_for_model("rs-06"));
  ASSERT_TRUE(decoded_status.has_value());
  ASSERT_GT(std::abs(decoded_status->velocity), 1.0F);
  ASSERT_TRUE(protocol.process_frame(status_frame,
                                     start + std::chrono::milliseconds(10)));

  double velocity = 0.0;
  ASSERT_TRUE(protocol.get_state("joint1", "velocity", velocity));
  EXPECT_DOUBLE_EQ(velocity, 0.0);

  position_reply.data = {0x19, 0x70, 0, 0, 0x00, 0x00, 0x00, 0x3F};
  ASSERT_TRUE(protocol.process_frame(position_reply,
                                     start + std::chrono::milliseconds(200)));
  ASSERT_TRUE(protocol.get_state("joint1", "velocity", velocity));
  EXPECT_NEAR(velocity, 2.5, 1e-9);

  status_frame.data[2] = 0xF0;
  ASSERT_TRUE(protocol.process_frame(status_frame,
                                     start + std::chrono::milliseconds(210)));
  ASSERT_TRUE(protocol.get_state("joint1", "velocity", velocity));
  EXPECT_NEAR(velocity, 2.5, 1e-9);

  position_reply.data = {0x19, 0x70, 0, 0, 0x00, 0x00, 0x80, 0x3F};
  ASSERT_TRUE(protocol.process_frame(position_reply,
                                     start + std::chrono::milliseconds(400)));
  ASSERT_TRUE(protocol.get_state("joint1", "velocity", velocity));
  EXPECT_NEAR(velocity, 2.5, 1e-9);
}

TEST(RobStridePlugin, RejectsInvalidMechanicalPositionFeedback) {
  rebot_robstride_protocol::RobStrideProtocol protocol;
  shared_can_interface::v1::DeviceConfiguration device;
  device.name = "joint1";
  device.command_interfaces = {"position"};
  device.state_interfaces = {"position", "velocity"};
  device.parameters = {{"motor_id", "1"},  {"model", "rs-06"},
                       {"direction", "1"}, {"offset", "0"},
                       {"mit_kp", "50"},   {"mit_kd", "3"}};
  protocol.configure({{"host_id", "0xFD"}}, {device});

  const auto start = std::chrono::steady_clock::now();
  codec::Frame position_reply;
  position_reply.arbitration_id = codec::build_extended_id(
      codec::CommunicationType::kReadParameter, 1, 0xFD);
  position_reply.data = {0x19, 0x70, 0, 0, 0x00, 0x00, 0x80, 0x7F};
  EXPECT_FALSE(protocol.process_frame(position_reply, start));
  EXPECT_FALSE(protocol.initial_state_ready());

  position_reply.data = {0x19, 0x70, 0, 0, 0x00, 0x00, 0xA0, 0x41};
  EXPECT_FALSE(protocol.process_frame(position_reply, start));
  EXPECT_FALSE(protocol.initial_state_ready());

  position_reply.data = {0x19, 0x70, 0, 0, 0, 0, 0, 0};
  ASSERT_TRUE(protocol.process_frame(position_reply, start));
  position_reply.data = {0x19, 0x70, 0, 0, 0x00, 0x00, 0x20, 0x41};
  EXPECT_FALSE(protocol.process_frame(position_reply,
                                      start + std::chrono::milliseconds(1)));
  double position = 1.0;
  double velocity = 1.0;
  ASSERT_TRUE(protocol.get_state("joint1", "position", position));
  ASSERT_TRUE(protocol.get_state("joint1", "velocity", velocity));
  EXPECT_DOUBLE_EQ(position, 0.0);
  EXPECT_DOUBLE_EQ(velocity, 0.0);
}
