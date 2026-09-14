#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string_view>

#include "shared_can_interface/can_frame.hpp"

namespace rebot_robstride_protocol::codec {

using Frame = shared_can_interface::v1::CanFrame;

namespace parameter {
constexpr std::uint16_t kRunMode = 0x7005;
constexpr std::uint16_t kMechanicalPosition = 0x7019;
constexpr std::uint16_t kFilteredCurrent = 0x701A;
} // namespace parameter

enum class CommunicationType : std::uint8_t {
  kGetDeviceId = 0,
  kOperationControl = 1,
  kOperationStatus = 2,
  kEnable = 3,
  kDisable = 4,
  kSetZeroPosition = 6,
  kSetDeviceId = 7,
  kReadParameter = 17,
  kWriteParameter = 18,
  kFaultReport = 21,
  kSaveParameters = 22,
  kActiveReport = 24,
  kSetProtocol = 25,
};

struct ExtendedId {
  CommunicationType communication_type{CommunicationType::kGetDeviceId};
  std::uint16_t extra_data{0};
  std::uint8_t node_id{0};
};

struct MotorLimits {
  float position_max{0.0F};
  float velocity_max{0.0F};
  float torque_max{0.0F};
  float kp_max{0.0F};
  float kd_max{0.0F};
};

struct Status {
  std::uint8_t device_id{0};
  std::uint8_t mode_state{0};
  float position{0.0F};
  float velocity{0.0F};
  float torque{0.0F};
  float temperature_c{0.0F};
  bool uncalibrated{false};
  bool stall{false};
  bool magnetic_encoder_fault{false};
  bool overtemperature{false};
  bool overcurrent{false};
  bool undervoltage{false};
};

struct ParameterResponse {
  std::uint8_t device_id{0};
  std::uint16_t parameter_id{0};
  std::array<std::uint8_t, 4> raw_value{};
};

std::uint32_t build_extended_id(CommunicationType type,
                                std::uint16_t extra_data, std::uint8_t node_id);
ExtendedId parse_extended_id(std::uint32_t arbitration_id);
MotorLimits limits_for_model(std::string_view model);

Frame make_ping(std::uint8_t motor_id, std::uint8_t host_id);
Frame make_enable(std::uint8_t motor_id, std::uint8_t host_id);
Frame make_disable(std::uint8_t motor_id, std::uint8_t host_id,
                   bool clear_error = false);
Frame make_read_parameter(std::uint8_t motor_id, std::uint8_t host_id,
                          std::uint16_t parameter_id);
Frame make_write_parameter_i8(std::uint8_t motor_id, std::uint8_t host_id,
                              std::uint16_t parameter_id, std::int8_t value);
Frame make_write_parameter_f32(std::uint8_t motor_id, std::uint8_t host_id,
                               std::uint16_t parameter_id, float value);
Frame make_mit_command(std::uint8_t motor_id, const MotorLimits &limits,
                       float position, float velocity, float kp, float kd,
                       float torque);

std::optional<Status> decode_status(const Frame &frame,
                                    const MotorLimits &limits);
std::optional<ParameterResponse> decode_parameter_response(const Frame &frame);
float parameter_as_f32(const ParameterResponse &response);

} // namespace rebot_robstride_protocol::codec
