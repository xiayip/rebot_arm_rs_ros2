#include <gtest/gtest.h>

#include <chrono>
#include <stdexcept>

#include "rebot_robstride_protocol/robstride_protocol.hpp"

namespace {
using namespace std::chrono_literals;
namespace codec = rebot_robstride_protocol::codec;
using Clock = std::chrono::steady_clock;

class StartupReadiness : public ::testing::Test {
protected:
  rebot_robstride_protocol::RobStrideProtocol protocol;
  Clock::time_point start = Clock::now();
  void SetUp() override {
    shared_can_interface::v1::DeviceConfiguration device;
    device.name = "joint4";
    device.command_interfaces = {"position", "kp", "kd"};
    device.state_interfaces = {"position", "temperature"};
    device.parameters = {{"motor_id", "4"},  {"model", "rs-00"},
                         {"direction", "1"}, {"offset", "0"},
                         {"mit_kp", "50"},   {"mit_kd", "5"}};
    protocol.configure({{"host_id", "0xFD"}}, {device});
    codec::Frame reply;
    reply.arbitration_id = codec::build_extended_id(
        codec::CommunicationType::kReadParameter, 4, 0xFD);
    for (const auto index : {0x7019U, 0x7005U, 0x701AU}) {
      reply.data = {};
      reply.data[0] = index & 255U;
      reply.data[1] = index >> 8U;
      ASSERT_TRUE(protocol.process_frame(reply, start));
    }
    ASSERT_TRUE(protocol.initial_state_ready());
    protocol.begin_enable_check(start);
  }
  void status(int ms, unsigned mode = 0, unsigned temperature = 340,
              unsigned q_raw = 32767, unsigned faults = 0, unsigned motor = 4) {
    codec::Frame frame;
    frame.arbitration_id = codec::build_extended_id(
        codec::CommunicationType::kOperationStatus,
        static_cast<std::uint16_t>((mode << 14U) | (faults << 8U) | motor),
        0xFD);
    frame.data = {static_cast<std::uint8_t>(q_raw >> 8U),
                  static_cast<std::uint8_t>(q_raw),
                  0x7F,
                  0xFF,
                  0x7F,
                  0xFF,
                  static_cast<std::uint8_t>(temperature >> 8U),
                  static_cast<std::uint8_t>(temperature)};
    protocol.process_frame(frame, start + std::chrono::milliseconds(ms));
  }
  bool ready(int ms) {
    return protocol.pending_enable_states(start + std::chrono::milliseconds(ms))
        .empty();
  }
};

TEST_F(StartupReadiness, ZeroStartupRepliesCannotEnable) {
  for (int ms : {0, 20, 40, 60, 80, 100})
    status(ms, 0, 0);
  EXPECT_FALSE(ready(101));
  EXPECT_THROW(protocol.enable_sequence(), std::runtime_error);
}

TEST_F(StartupReadiness, ZeroToStableFeedbackReproducesColdCapture) {
  for (int ms : {0, 20, 40, 60, 80, 100})
    status(ms, 0, 0);
  status(120);
  EXPECT_FALSE(ready(120));
  status(140);
  EXPECT_FALSE(ready(140));
  status(160);
  EXPECT_TRUE(ready(160));
  EXPECT_FALSE(ready(261));
}

TEST_F(StartupReadiness, ThreeBufferedRepliesDoNotSatisfyTimeWindow) {
  status(0);
  status(1);
  status(2);
  EXPECT_FALSE(ready(2));
  status(40);
  EXPECT_TRUE(ready(40));
}

TEST_F(StartupReadiness, FreshWarmMotorNeedsNoSettlingOrRedundantEnable) {
  // Use timestamps in the past so enable_sequence's real clock freshness
  // check does not depend on sleeping in this unit test.
  start = Clock::now() - 1ms;
  protocol.begin_enable_check(start);
  ASSERT_TRUE(protocol.set_command("joint4", "position", 0.012));
  status(0, 2);
  EXPECT_TRUE(ready(0));
  double baseline = 0;
  ASSERT_TRUE(protocol.get_initial_command("joint4", "position", baseline));
  EXPECT_DOUBLE_EQ(baseline, 0.012);
  const auto sequence = protocol.enable_sequence();
  ASSERT_EQ(sequence.size(), 1U);
  ASSERT_EQ(sequence[0].frames.size(), 1U);
  EXPECT_EQ(codec::parse_extended_id(sequence[0].frames[0].arbitration_id)
                .communication_type,
            codec::CommunicationType::kOperationControl);
}

TEST_F(StartupReadiness, ColdHoldReseedsLatestPoseInsteadOfOldPreload) {
  ASSERT_TRUE(protocol.set_command("joint4", "position", 0.012));
  status(0);
  status(20);
  status(40);
  EXPECT_TRUE(ready(40));
  double baseline = 123, measured = 0;
  ASSERT_TRUE(protocol.get_initial_command("joint4", "position", baseline));
  ASSERT_TRUE(protocol.get_state("joint4", "position", measured));
  EXPECT_DOUBLE_EQ(baseline, measured);
  const auto probes = protocol.pre_enable_sequence();
  ASSERT_EQ(probes.size(), 1U);
  const auto expected =
      codec::make_mit_command(4, codec::limits_for_model("rs-00"),
                              static_cast<float>(measured), 0, 50, 5, 0);
  EXPECT_EQ(probes[0].frames[0].data, expected.data);
}

TEST_F(StartupReadiness, FaultOrCalibrationBlocksEvenPreviouslyStableMotor) {
  status(0);
  status(20);
  status(40);
  EXPECT_TRUE(ready(40));
  status(60, 0, 340, 32767, 1);
  EXPECT_FALSE(ready(60));
  status(80, 1);
  EXPECT_FALSE(ready(80));
  status(100);
  EXPECT_FALSE(ready(100));
}

TEST_F(StartupReadiness, LargeMotionRequiresNewActivation) {
  status(0);
  status(20);
  status(40, 0, 340, 33000);
  status(60);
  status(80);
  status(100);
  EXPECT_FALSE(ready(100));
}

TEST_F(StartupReadiness, SmallMotionAndTemperatureJumpResetStableWindow) {
  status(0);
  status(20);
  status(40, 0, 360);
  EXPECT_FALSE(ready(40));
  status(60, 0, 360);
  status(80, 0, 360);
  EXPECT_TRUE(ready(80));
  status(100, 0, 360, 32780);
  EXPECT_FALSE(ready(100));
}

TEST_F(StartupReadiness, MissingGapAndOtherMotorNeverCountAsFreshReplies) {
  status(0);
  status(20);
  status(40, 0, 340, 32767, 0, 5);
  EXPECT_FALSE(ready(40));
  status(200);
  EXPECT_FALSE(ready(200));
  status(220);
  status(240);
  EXPECT_TRUE(ready(240));
  protocol.begin_enable_check(start + 250ms);
  EXPECT_FALSE(ready(250));
}

TEST(RobStrideStartupCodec, ReadErrorReplyCannotSeedInitialPosition) {
  codec::Frame frame;
  frame.arbitration_id = codec::build_extended_id(
      codec::CommunicationType::kReadParameter, 0x0104, 0xFD);
  frame.data = {0x19, 0x70, 0, 0, 0, 0, 0, 0};
  EXPECT_FALSE(codec::decode_parameter_response(frame).has_value());
}

TEST_F(StartupReadiness, ReplayedTimestampCannotAdvanceStableWindow) {
  status(0);
  status(20);
  status(20);
  EXPECT_FALSE(ready(20));
  status(40);
  EXPECT_TRUE(ready(40));
  status(10, 0, 0);
  EXPECT_TRUE(ready(40));
}

TEST(RobStrideStartup, MixedSixMotorsOnlyEnablesReadyColdJoint4) {
  rebot_robstride_protocol::RobStrideProtocol protocol;
  std::vector<shared_can_interface::v1::DeviceConfiguration> devices;
  for (int id = 1; id <= 6; ++id) {
    shared_can_interface::v1::DeviceConfiguration device;
    device.name = "joint" + std::to_string(id);
    device.command_interfaces = {"position", "kp", "kd"};
    device.state_interfaces = {"position"};
    device.parameters = {{"motor_id", std::to_string(id)},
                         {"model", "rs-00"},
                         {"direction", "1"},
                         {"offset", "0"},
                         {"mit_kp", "50"},
                         {"mit_kd", "5"}};
    devices.push_back(device);
  }
  protocol.configure({{"host_id", "0xFD"}}, devices);
  const auto start = Clock::now() - 200ms;
  for (unsigned id = 1; id <= 6; ++id) {
    codec::Frame frame;
    frame.arbitration_id = codec::build_extended_id(
        codec::CommunicationType::kReadParameter, id, 0xFD);
    for (unsigned index : {0x7019, 0x7005, 0x701A}) {
      frame.data = {};
      frame.data[0] = index & 255U;
      frame.data[1] = index >> 8U;
      ASSERT_TRUE(protocol.process_frame(frame, start));
    }
    ASSERT_TRUE(
        protocol.set_command("joint" + std::to_string(id), "position", 0.01));
  }
  protocol.begin_enable_check(start);
  const auto receive = [&](unsigned id, int ms, unsigned temperature) {
    codec::Frame frame;
    frame.arbitration_id = codec::build_extended_id(
        codec::CommunicationType::kOperationStatus,
        static_cast<std::uint16_t>(((id == 4 ? 0U : 2U) << 14U) | id), 0xFD);
    frame.data = {0x7F,
                  0xFF,
                  0x7F,
                  0xFF,
                  0x7F,
                  0xFF,
                  static_cast<std::uint8_t>(temperature >> 8U),
                  static_cast<std::uint8_t>(temperature)};
    ASSERT_TRUE(
        protocol.process_frame(frame, start + std::chrono::milliseconds(ms)));
  };
  for (unsigned id = 1; id <= 6; ++id)
    receive(id, 0, id == 4 ? 0 : 340);
  const auto pending = protocol.pending_enable_states(start + 1ms);
  ASSERT_EQ(pending.size(), 1U);
  EXPECT_NE(pending[0].find("joint4"), std::string::npos);
  EXPECT_THROW(protocol.enable_sequence(), std::runtime_error);
  // Keep warm feedback fresh throughout the cold joint's readiness window.
  for (int ms : {120, 140, 160}) {
    for (unsigned id = 1; id <= 6; ++id)
      receive(id, ms, 340);
  }
  EXPECT_TRUE(protocol.pending_enable_states(start + 161ms).empty());
  for (unsigned id = 1; id <= 6; ++id) {
    const auto name = "joint" + std::to_string(id);
    double command = 0;
    ASSERT_TRUE(protocol.get_initial_command(name, "position", command));
    EXPECT_NEAR(command, id == 4 ? 0.0 : 0.01, 1e-6);
    ASSERT_TRUE(protocol.set_command(name, "position", command));
  }
  const auto sequence = protocol.enable_sequence();
  ASSERT_EQ(sequence.size(), 1U);
  ASSERT_EQ(sequence[0].frames.size(), 7U);
  unsigned enable_count = 0;
  for (std::size_t i = 0; i < sequence[0].frames.size(); ++i) {
    const auto decoded =
        codec::parse_extended_id(sequence[0].frames[i].arbitration_id);
    if (decoded.communication_type == codec::CommunicationType::kEnable) {
      ++enable_count;
      EXPECT_EQ(decoded.node_id, 4U);
      ASSERT_LT(i + 1, sequence[0].frames.size());
      const auto next =
          codec::parse_extended_id(sequence[0].frames[i + 1].arbitration_id);
      EXPECT_EQ(next.communication_type,
                codec::CommunicationType::kOperationControl);
      EXPECT_EQ(next.node_id, 4U);
    } else {
      EXPECT_EQ(decoded.communication_type,
                codec::CommunicationType::kOperationControl);
    }
  }
  EXPECT_EQ(enable_count, 1U);
}
} // namespace
