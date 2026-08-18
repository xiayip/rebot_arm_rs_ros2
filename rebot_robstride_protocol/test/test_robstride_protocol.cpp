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
  device.command_interfaces = {"position", "velocity", "kp", "kd", "t_ff"};
  device.state_interfaces = {"position", "velocity", "effort"};
  device.parameters = {{"motor_id", "1"},  {"model", "rs-06"},
                       {"direction", "1"}, {"offset", "0"},
                       {"mit_kp", "50"},   {"mit_kd", "3"}};
  protocol.configure(bus, {device});

  const auto requests = protocol.initial_state_requests();
  ASSERT_EQ(requests.size(), 3U);
  EXPECT_EQ(requests[0].data[0], 0x19);
  EXPECT_EQ(requests[0].data[1], 0x70);
  EXPECT_EQ(requests[1].data[0], 0x05);
  EXPECT_EQ(requests[1].data[1], 0x70);
  EXPECT_EQ(requests[2].data[0], 0x1A);
  EXPECT_EQ(requests[2].data[1], 0x70);

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
  EXPECT_FALSE(protocol.initial_state_ready());

  reply.data = {0x05, 0x70, 0, 0, 0, 0, 0, 0};
  EXPECT_TRUE(protocol.process_frame(reply, std::chrono::steady_clock::now()));
  EXPECT_FALSE(protocol.initial_state_ready());

  reply.data = {0x1A, 0x70, 0, 0, 0, 0, 0, 0};
  EXPECT_TRUE(protocol.process_frame(reply, std::chrono::steady_clock::now()));
  EXPECT_TRUE(protocol.initial_state_ready());
  double position = 1.0;
  EXPECT_TRUE(protocol.get_state("joint1", "position", position));
  EXPECT_DOUBLE_EQ(position, 0.0);
  EXPECT_TRUE(protocol.set_command("joint1", "position", 0.25));
  EXPECT_TRUE(protocol.set_command("joint1", "velocity", 1.5));
  EXPECT_TRUE(protocol.set_command("joint1", "t_ff", 2.0));
  EXPECT_TRUE(protocol.set_command("joint1", "kp", 60.0));
  EXPECT_TRUE(protocol.set_command("joint1", "kd", 4.0));
  EXPECT_FALSE(protocol.set_command("joint1", "current", 0.25));
  std::vector<shared_can_interface::v1::CanFrame> commands;
  protocol.command_frames(commands);
  ASSERT_EQ(commands.size(), 1U);
  const auto expected = codec::make_mit_command(
      1, codec::limits_for_model("rs-06"), 0.25F, 1.5F, 60.0F, 4.0F, 2.0F);
  EXPECT_EQ(commands.front().arbitration_id, expected.arbitration_id);
  EXPECT_EQ(commands.front().data, expected.data);
  EXPECT_EQ(protocol.command_frame_capacity(), 1U);
}

TEST(RobStridePlugin, SkipsRedundantEnableWhenMotorIsAlreadyRunning) {
  rebot_robstride_protocol::RobStrideProtocol protocol;
  shared_can_interface::v1::DeviceConfiguration device;
  device.name = "joint1";
  device.command_interfaces = {"position", "velocity", "kp", "kd", "t_ff"};
  device.state_interfaces = {"position"};
  device.parameters = {{"motor_id", "1"},  {"model", "rs-06"},
                       {"direction", "1"}, {"offset", "0"},
                       {"mit_kp", "50"},   {"mit_kd", "3"}};
  protocol.configure({{"host_id", "0xFD"}}, {device});

  codec::Frame reply;
  reply.arbitration_id = codec::build_extended_id(
      codec::CommunicationType::kReadParameter, 1, 0xFD);
  reply.data = {0x19, 0x70, 0, 0, 0, 0, 0, 0};
  ASSERT_TRUE(protocol.process_frame(reply, std::chrono::steady_clock::now()));
  reply.data = {0x05, 0x70, 0, 0, 0, 0, 0, 0};
  ASSERT_TRUE(protocol.process_frame(reply, std::chrono::steady_clock::now()));
  reply.data = {0x1A, 0x70, 0, 0, 0, 0, 0, 0x40};
  ASSERT_TRUE(protocol.process_frame(reply, std::chrono::steady_clock::now()));

  double startup_position = 0.0;
  ASSERT_TRUE(
      protocol.get_initial_command("joint1", "position", startup_position));
  EXPECT_NEAR(startup_position, 2.0 / std::sqrt(2.0) * 1.09 / 50.0, 1e-9);
  ASSERT_TRUE(protocol.set_command("joint1", "position", startup_position));

  const auto probe = protocol.pre_enable_sequence();
  ASSERT_EQ(probe.size(), 1U);
  EXPECT_EQ(probe[0].delay_after, std::chrono::milliseconds(10));
  ASSERT_EQ(probe[0].frames.size(), 1U);
  EXPECT_EQ(codec::parse_extended_id(probe[0].frames[0].arbitration_id)
                .communication_type,
            codec::CommunicationType::kOperationControl);

  auto running_status = probe[0].frames[0];
  running_status.arbitration_id = codec::build_extended_id(
      codec::CommunicationType::kOperationStatus,
      static_cast<std::uint16_t>((2U << 14U) | 1U), 0xFD);
  ASSERT_TRUE(
      protocol.process_frame(running_status, std::chrono::steady_clock::now()));

  const auto sequence = protocol.enable_sequence();
  ASSERT_EQ(sequence.size(), 1U);
  EXPECT_EQ(sequence[0].delay_after, std::chrono::milliseconds(0));
  ASSERT_EQ(sequence[0].frames.size(), 1U);
  EXPECT_EQ(codec::parse_extended_id(sequence[0].frames[0].arbitration_id)
                .communication_type,
            codec::CommunicationType::kOperationControl);
}

TEST(RobStridePlugin, CorrectsNonMitModeBeforeEnabling) {
  rebot_robstride_protocol::RobStrideProtocol protocol;
  shared_can_interface::v1::DeviceConfiguration device;
  device.name = "joint1";
  device.command_interfaces = {"position"};
  device.state_interfaces = {"position"};
  device.parameters = {{"motor_id", "1"},  {"model", "rs-06"},
                       {"direction", "1"}, {"offset", "0"},
                       {"mit_kp", "50"},   {"mit_kd", "3"}};
  protocol.configure({{"host_id", "0xFD"}}, {device});

  codec::Frame reply;
  reply.arbitration_id = codec::build_extended_id(
      codec::CommunicationType::kReadParameter, 1, 0xFD);
  reply.data = {0x19, 0x70, 0, 0, 0, 0, 0, 0};
  ASSERT_TRUE(protocol.process_frame(reply, std::chrono::steady_clock::now()));
  reply.data = {0x05, 0x70, 0, 0, 1, 0, 0, 0};
  ASSERT_TRUE(protocol.process_frame(reply, std::chrono::steady_clock::now()));
  reply.data = {0x1A, 0x70, 0, 0, 0, 0, 0, 0};
  ASSERT_TRUE(protocol.process_frame(reply, std::chrono::steady_clock::now()));

  EXPECT_TRUE(protocol.pre_enable_sequence().empty());
  const auto sequence = protocol.enable_sequence();
  ASSERT_EQ(sequence.size(), 2U);
  EXPECT_EQ(sequence[0].delay_after, std::chrono::milliseconds(40));
  ASSERT_EQ(sequence[0].frames.size(), 1U);
  const auto &set_mode = sequence[0].frames[0];
  EXPECT_EQ(
      codec::parse_extended_id(set_mode.arbitration_id).communication_type,
      codec::CommunicationType::kWriteParameter);
  EXPECT_EQ(set_mode.data[0], 0x05);
  EXPECT_EQ(set_mode.data[1], 0x70);
  EXPECT_EQ(set_mode.data[4], 0U);
  ASSERT_EQ(sequence[1].frames.size(), 2U);
}

TEST(RobStridePlugin, AppliesJointDirectionToMitMotionAndFeedForward) {
  rebot_robstride_protocol::RobStrideProtocol protocol;
  shared_can_interface::v1::DeviceConfiguration device;
  device.name = "joint1";
  device.command_interfaces = {"position", "velocity", "kp", "kd", "t_ff"};
  device.state_interfaces = {"position"};
  device.parameters = {{"motor_id", "1"},   {"model", "rs-06"},
                       {"direction", "-1"}, {"offset", "0.1"},
                       {"mit_kp", "50"},    {"mit_kd", "3"}};
  protocol.configure({{"host_id", "0xFD"}}, {device});

  ASSERT_TRUE(protocol.set_command("joint1", "position", 0.25));
  ASSERT_TRUE(protocol.set_command("joint1", "velocity", 1.5));
  ASSERT_TRUE(protocol.set_command("joint1", "t_ff", 2.0));
  ASSERT_TRUE(protocol.set_command("joint1", "kp", 60.0));
  ASSERT_TRUE(protocol.set_command("joint1", "kd", 4.0));

  std::vector<shared_can_interface::v1::CanFrame> commands;
  protocol.command_frames(commands);
  ASSERT_EQ(commands.size(), 1U);
  const auto expected = codec::make_mit_command(
      1, codec::limits_for_model("rs-06"), -0.15F, -1.5F, 60.0F, 4.0F, -2.0F);
  EXPECT_EQ(commands.front().arbitration_id, expected.arbitration_id);
  EXPECT_EQ(commands.front().data, expected.data);
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

TEST(RobStridePlugin, UsesOperationStatusAsTheOnlyRuntimeStateSource) {
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

  double velocity = 0.0;
  EXPECT_FALSE(protocol.get_state("joint1", "velocity", velocity));
  EXPECT_FALSE(protocol.periodic_request().has_value());

  auto status_frame = codec::make_mit_command(
      1, codec::limits_for_model("rs-06"), 0.25F, 3.5F, 0.0F, 0.0F, 0.0F);
  status_frame.arbitration_id = codec::build_extended_id(
      codec::CommunicationType::kOperationStatus, 1, 0xFD);
  ASSERT_TRUE(protocol.process_frame(status_frame,
                                     start + std::chrono::milliseconds(5)));

  double position = 0.0;
  ASSERT_TRUE(protocol.get_state("joint1", "position", position));
  ASSERT_TRUE(protocol.get_state("joint1", "velocity", velocity));
  EXPECT_NEAR(position, 0.25, 4e-4);
  EXPECT_NEAR(velocity, 3.5, 0.002);

  // Later diagnostic parameter responses must not overwrite live state.
  position_reply.data = {0x19, 0x70, 0, 0, 0x00, 0x00, 0x00, 0x3F};
  ASSERT_TRUE(protocol.process_frame(position_reply,
                                     start + std::chrono::milliseconds(50)));
  ASSERT_TRUE(protocol.get_state("joint1", "position", position));
  ASSERT_TRUE(protocol.get_state("joint1", "velocity", velocity));
  EXPECT_NEAR(position, 0.25, 4e-4);
  EXPECT_NEAR(velocity, 3.5, 0.002);
}

TEST(RobStridePlugin, AppliesDirectionAndOffsetToStatusFeedback) {
  rebot_robstride_protocol::RobStrideProtocol protocol;
  shared_can_interface::v1::DeviceConfiguration device;
  device.name = "joint1";
  device.command_interfaces = {"position"};
  device.state_interfaces = {"position", "velocity"};
  device.parameters = {{"motor_id", "1"},   {"model", "rs-06"},
                       {"direction", "-1"}, {"offset", "0.1"},
                       {"mit_kp", "50"},    {"mit_kd", "3"}};
  protocol.configure({{"host_id", "0xFD"}}, {device});

  auto status_frame = codec::make_mit_command(
      1, codec::limits_for_model("rs-06"), 0.24F, 1.5F, 0.0F, 0.0F, 0.0F);
  status_frame.arbitration_id = codec::build_extended_id(
      codec::CommunicationType::kOperationStatus, 1, 0xFD);
  ASSERT_TRUE(
      protocol.process_frame(status_frame, std::chrono::steady_clock::now()));

  double position = 0.0;
  double velocity = 0.0;
  ASSERT_TRUE(protocol.get_state("joint1", "position", position));
  ASSERT_TRUE(protocol.get_state("joint1", "velocity", velocity));
  EXPECT_NEAR(position, -0.14, 4e-4);
  EXPECT_NEAR(velocity, -1.5, 0.002);
}

TEST(RobStridePlugin, ValidatesOnlyTheInitialMechanicalPositionSnapshot) {
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
  position_reply.data = {0x19, 0x70, 0, 0, 0x00, 0x00, 0xA0, 0x3F};
  EXPECT_TRUE(protocol.process_frame(position_reply,
                                     start + std::chrono::milliseconds(1)));
  double position = 1.0;
  double velocity = 1.0;
  ASSERT_TRUE(protocol.get_state("joint1", "position", position));
  EXPECT_FALSE(protocol.get_state("joint1", "velocity", velocity));
  EXPECT_DOUBLE_EQ(position, 0.0);
  EXPECT_DOUBLE_EQ(velocity, 1.0);
}
