#include <gtest/gtest.h>

#include <array>
#include <cmath>

#include "rebot_arm_rs_hardware/robstride_protocol.hpp"

namespace rs = rebot_arm_rs_hardware::robstride;

TEST(RobStrideProtocol, ExtendedIdRoundTrip) {
  const auto encoded = rs::build_extended_id(rs::CommunicationType::kReadParameter, 0xABCD, 0x7F);
  const auto decoded = rs::parse_extended_id(encoded);
  EXPECT_EQ(decoded.communication_type, rs::CommunicationType::kReadParameter);
  EXPECT_EQ(decoded.extra_data, 0xABCD);
  EXPECT_EQ(decoded.node_id, 0x7F);
}

TEST(RobStrideProtocol, ParameterReadUsesLittleEndianIndex) {
  const auto frame = rs::make_read_parameter(0x03, 0xFD, 0x7019);
  EXPECT_EQ(frame.arbitration_id, 0x1100FD03U);
  EXPECT_EQ(frame.data[0], 0x19);
  EXPECT_EQ(frame.data[1], 0x70);
}

TEST(RobStrideProtocol, FloatWriteUsesLittleEndianValue) {
  const auto frame = rs::make_write_parameter_f32(0x04, 0xFD, 0x7016, 1.25F);
  EXPECT_EQ(frame.arbitration_id, 0x1200FD04U);
  EXPECT_EQ(frame.data[0], 0x16);
  EXPECT_EQ(frame.data[1], 0x70);
  EXPECT_EQ(frame.data[4], 0x00);
  EXPECT_EQ(frame.data[5], 0x00);
  EXPECT_EQ(frame.data[6], 0xA0);
  EXPECT_EQ(frame.data[7], 0x3F);
}

TEST(RobStrideProtocol, MitCommandSeparatesTorqueIntoExtendedId) {
  const auto limits = rs::limits_for_model("rs-00");
  const auto base = rs::make_mit_command(6, limits, 0.1F, 0.2F, 3.0F, 0.4F, 0.5F);
  const auto torque_changed = rs::make_mit_command(6, limits, 0.1F, 0.2F, 3.0F, 0.4F, 0.8F);
  EXPECT_EQ(base.data, torque_changed.data);
  EXPECT_NE(rs::parse_extended_id(base.arbitration_id).extra_data,
            rs::parse_extended_id(torque_changed.arbitration_id).extra_data);
}

TEST(RobStrideProtocol, StatusDecodeUsesModelLimitsAndFlags) {
  rs::Frame frame;
  frame.arbitration_id = rs::build_extended_id(
      rs::CommunicationType::kOperationStatus,
      static_cast<std::uint16_t>((2U << 14U) | (1U << 13U) | (1U << 9U) | 0x03U), 0xFD);
  frame.data = {0x80, 0x00, 0x80, 0x00, 0x80, 0x00, 0x01, 0x2C};
  const auto status = rs::decode_status(frame, rs::limits_for_model("rs-06"));
  ASSERT_TRUE(status.has_value());
  EXPECT_EQ(status->device_id, 3);
  EXPECT_EQ(status->mode_state, 2);
  EXPECT_TRUE(status->uncalibrated);
  EXPECT_TRUE(status->overcurrent);
  EXPECT_NEAR(status->temperature_c, 30.0F, 1e-5F);
}

TEST(RobStrideProtocol, ParameterResponseDecodesMotorAndFloat) {
  rs::Frame frame;
  frame.arbitration_id = rs::build_extended_id(rs::CommunicationType::kReadParameter, 0x0005, 0xFD);
  frame.data = {0x19, 0x70, 0, 0, 0, 0, 0x20, 0x40};
  const auto response = rs::decode_parameter_response(frame);
  ASSERT_TRUE(response.has_value());
  EXPECT_EQ(response->device_id, 5);
  EXPECT_EQ(response->parameter_id, 0x7019);
  EXPECT_FLOAT_EQ(rs::parameter_as_f32(*response), 2.5F);
}

