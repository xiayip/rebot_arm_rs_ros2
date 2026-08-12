#include "rebot_arm_rs_hardware/robstride_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace rebot_arm_rs_hardware::robstride {
namespace {

constexpr float kPi = 3.14159265358979323846F;

std::uint16_t encode_symmetric(float value, float maximum) {
  const float clamped = std::clamp(value, -maximum, maximum);
  return static_cast<std::uint16_t>(((clamped / maximum) + 1.0F) * 32767.0F);
}

std::uint16_t encode_positive(float value, float maximum) {
  const float clamped = std::clamp(value, 0.0F, maximum);
  return static_cast<std::uint16_t>((clamped / maximum) * 65535.0F);
}

float decode_symmetric(std::uint16_t value, float maximum) {
  return (static_cast<float>(value) / 32767.0F - 1.0F) * maximum;
}

Frame make_frame(CommunicationType type, std::uint16_t extra_data, std::uint8_t motor_id) {
  Frame frame;
  frame.arbitration_id = build_extended_id(type, extra_data, motor_id);
  return frame;
}

}  // namespace

std::uint32_t build_extended_id(CommunicationType type, std::uint16_t extra_data,
                                std::uint8_t node_id) {
  return (static_cast<std::uint32_t>(type) << 24U) |
         (static_cast<std::uint32_t>(extra_data) << 8U) | node_id;
}

ExtendedId parse_extended_id(std::uint32_t arbitration_id) {
  return {
      static_cast<CommunicationType>((arbitration_id >> 24U) & 0x1FU),
      static_cast<std::uint16_t>((arbitration_id >> 8U) & 0xFFFFU),
      static_cast<std::uint8_t>(arbitration_id & 0xFFU),
  };
}

MotorLimits limits_for_model(std::string_view model) {
  if (model == "rs-06") {
    return {4.0F * kPi, 50.0F, 36.0F, 5000.0F, 100.0F};
  }
  if (model == "rs-00") {
    return {4.0F * kPi, 33.0F, 14.0F, 500.0F, 5.0F};
  }
  throw std::invalid_argument("unsupported RobStride model");
}

Frame make_ping(std::uint8_t motor_id, std::uint8_t host_id) {
  return make_frame(CommunicationType::kGetDeviceId, host_id, motor_id);
}

Frame make_enable(std::uint8_t motor_id, std::uint8_t host_id) {
  return make_frame(CommunicationType::kEnable, host_id, motor_id);
}

Frame make_disable(std::uint8_t motor_id, std::uint8_t host_id, bool clear_error) {
  auto frame = make_frame(CommunicationType::kDisable, host_id, motor_id);
  frame.data[0] = clear_error ? 1U : 0U;
  return frame;
}

Frame make_read_parameter(std::uint8_t motor_id, std::uint8_t host_id,
                          std::uint16_t parameter_id) {
  auto frame = make_frame(CommunicationType::kReadParameter, host_id, motor_id);
  frame.data[0] = static_cast<std::uint8_t>(parameter_id & 0xFFU);
  frame.data[1] = static_cast<std::uint8_t>(parameter_id >> 8U);
  return frame;
}

Frame make_write_parameter_i8(std::uint8_t motor_id, std::uint8_t host_id,
                              std::uint16_t parameter_id, std::int8_t value) {
  auto frame = make_frame(CommunicationType::kWriteParameter, host_id, motor_id);
  frame.data[0] = static_cast<std::uint8_t>(parameter_id & 0xFFU);
  frame.data[1] = static_cast<std::uint8_t>(parameter_id >> 8U);
  frame.data[4] = static_cast<std::uint8_t>(value);
  return frame;
}

Frame make_write_parameter_f32(std::uint8_t motor_id, std::uint8_t host_id,
                               std::uint16_t parameter_id, float value) {
  auto frame = make_frame(CommunicationType::kWriteParameter, host_id, motor_id);
  frame.data[0] = static_cast<std::uint8_t>(parameter_id & 0xFFU);
  frame.data[1] = static_cast<std::uint8_t>(parameter_id >> 8U);
  std::array<std::uint8_t, 4> raw{};
  std::memcpy(raw.data(), &value, sizeof(value));
  std::copy(raw.begin(), raw.end(), frame.data.begin() + 4);
  return frame;
}

Frame make_mit_command(std::uint8_t motor_id, const MotorLimits &limits, float position,
                       float velocity, float kp, float kd, float torque) {
  const auto position_raw = encode_symmetric(position, limits.position_max);
  const auto velocity_raw = encode_symmetric(velocity, limits.velocity_max);
  const auto kp_raw = encode_positive(kp, limits.kp_max);
  const auto kd_raw = encode_positive(kd, limits.kd_max);
  const auto torque_raw = encode_symmetric(torque, limits.torque_max);

  auto frame = make_frame(CommunicationType::kOperationControl, torque_raw, motor_id);
  frame.data = {
      static_cast<std::uint8_t>(position_raw >> 8U), static_cast<std::uint8_t>(position_raw),
      static_cast<std::uint8_t>(velocity_raw >> 8U), static_cast<std::uint8_t>(velocity_raw),
      static_cast<std::uint8_t>(kp_raw >> 8U), static_cast<std::uint8_t>(kp_raw),
      static_cast<std::uint8_t>(kd_raw >> 8U), static_cast<std::uint8_t>(kd_raw),
  };
  return frame;
}

std::optional<Status> decode_status(const Frame &frame, const MotorLimits &limits) {
  if (!frame.extended || frame.dlc != 8U) {
    return std::nullopt;
  }
  const auto id = parse_extended_id(frame.arbitration_id);
  if (id.communication_type != CommunicationType::kOperationStatus) {
    return std::nullopt;
  }

  const auto position = static_cast<std::uint16_t>((frame.data[0] << 8U) | frame.data[1]);
  const auto velocity = static_cast<std::uint16_t>((frame.data[2] << 8U) | frame.data[3]);
  const auto torque = static_cast<std::uint16_t>((frame.data[4] << 8U) | frame.data[5]);
  const auto temperature = static_cast<std::uint16_t>((frame.data[6] << 8U) | frame.data[7]);

  return Status{
      static_cast<std::uint8_t>(id.extra_data & 0xFFU),
      static_cast<std::uint8_t>((id.extra_data >> 14U) & 0x03U),
      decode_symmetric(position, limits.position_max),
      decode_symmetric(velocity, limits.velocity_max),
      decode_symmetric(torque, limits.torque_max),
      static_cast<float>(temperature) * 0.1F,
      (id.extra_data & (1U << 13U)) != 0U,
      (id.extra_data & (1U << 12U)) != 0U,
      (id.extra_data & (1U << 11U)) != 0U,
      (id.extra_data & (1U << 10U)) != 0U,
      (id.extra_data & (1U << 9U)) != 0U,
      (id.extra_data & (1U << 8U)) != 0U,
  };
}

std::optional<ParameterResponse> decode_parameter_response(const Frame &frame) {
  if (!frame.extended || frame.dlc != 8U) {
    return std::nullopt;
  }
  const auto id = parse_extended_id(frame.arbitration_id);
  if (id.communication_type != CommunicationType::kReadParameter) {
    return std::nullopt;
  }
  return ParameterResponse{
      static_cast<std::uint8_t>(id.extra_data & 0xFFU),
      static_cast<std::uint16_t>(frame.data[0] | (frame.data[1] << 8U)),
      {frame.data[4], frame.data[5], frame.data[6], frame.data[7]},
  };
}

float parameter_as_f32(const ParameterResponse &response) {
  float value = 0.0F;
  std::memcpy(&value, response.raw_value.data(), sizeof(value));
  return value;
}

}  // namespace rebot_arm_rs_hardware::robstride
