#include "rebot_robstride_protocol/robstride_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace rebot_robstride_protocol {
namespace {

using codec::parameter::kFilteredCurrent;
using codec::parameter::kMechanicalPosition;
using codec::parameter::kRunMode;
constexpr double kPeakToRms = 0.70710678118654752440;
constexpr auto kStartupFeedbackFreshness = std::chrono::milliseconds(100);
constexpr auto kStartupStableDuration = std::chrono::milliseconds(30);
constexpr double kStartupStablePositionSpan = 0.003;
constexpr double kStartupMaximumDisplacement = 0.025;

double torque_constant_for_model(const std::string &model) {
  if (model == "rs-06") {
    return 1.09;
  }
  if (model == "rs-00") {
    return 1.48;
  }
  throw std::invalid_argument("unsupported RobStride model: " + model);
}

const std::string &
required_parameter(const shared_can_interface::v1::ParameterMap &parameters,
                   const std::string &name) {
  const auto iterator = parameters.find(name);
  if (iterator == parameters.end() || iterator->second.empty()) {
    throw std::invalid_argument("missing parameter: " + name);
  }
  return iterator->second;
}

std::uint8_t parse_u8(const std::string &value) {
  const auto parsed = std::stoul(value, nullptr, 0);
  if (parsed > 0xFFU) {
    throw std::out_of_range("8-bit value out of range: " + value);
  }
  return static_cast<std::uint8_t>(parsed);
}

} // namespace

void RobStrideProtocol::configure(
    const shared_can_interface::v1::ParameterMap &bus_parameters,
    const std::vector<shared_can_interface::v1::DeviceConfiguration> &devices) {
  host_id_ = parse_u8(required_parameter(bus_parameters, "host_id"));
  if (devices.empty()) {
    throw std::invalid_argument(
        "RobStride protocol requires at least one device");
  }

  std::set<std::uint8_t> motor_ids;
  devices_.clear();
  devices_.reserve(devices.size());
  for (const auto &configuration : devices) {
    for (const auto &interface_name : configuration.command_interfaces) {
      if (interface_name != "position" && interface_name != "velocity" &&
          interface_name != "kp" && interface_name != "kd" &&
          interface_name != "t_ff") {
        throw std::invalid_argument(
            configuration.name +
            " has unsupported RobStride command interface: " + interface_name);
      }
    }
    for (const auto &interface_name : configuration.state_interfaces) {
      if (interface_name != "position" && interface_name != "velocity" &&
          interface_name != "effort" && interface_name != "temperature") {
        throw std::invalid_argument(
            configuration.name +
            " has unsupported RobStride state interface: " + interface_name);
      }
    }
    Device device;
    device.name = configuration.name;
    device.motor_id =
        parse_u8(required_parameter(configuration.parameters, "motor_id"));
    if (!motor_ids.insert(device.motor_id).second) {
      throw std::invalid_argument("duplicate RobStride motor ID: " +
                                  std::to_string(device.motor_id));
    }
    const auto model = required_parameter(configuration.parameters, "model");
    device.limits = codec::limits_for_model(model);
    device.torque_constant_nm_per_arms = torque_constant_for_model(model);
    if (const auto torque_constant =
            configuration.parameters.find("torque_constant_nm_per_arms");
        torque_constant != configuration.parameters.end()) {
      device.torque_constant_nm_per_arms = std::stod(torque_constant->second);
    }
    if (const auto preload_max =
            configuration.parameters.find("startup_preload_max");
        preload_max != configuration.parameters.end()) {
      device.startup_preload_max = std::stod(preload_max->second);
    }
    device.direction =
        std::stod(required_parameter(configuration.parameters, "direction"));
    device.offset =
        std::stod(required_parameter(configuration.parameters, "offset"));
    device.mit_kp =
        std::stod(required_parameter(configuration.parameters, "mit_kp"));
    device.mit_kd =
        std::stod(required_parameter(configuration.parameters, "mit_kd"));
    if (std::abs(std::abs(device.direction) - 1.0) > 1e-9) {
      throw std::invalid_argument(device.name + " direction must be +1 or -1");
    }
    if (!std::isfinite(device.mit_kp) || device.mit_kp < 0.0 ||
        device.mit_kp > device.limits.kp_max) {
      throw std::invalid_argument(device.name +
                                  " mit_kp is outside model limits");
    }
    if (!std::isfinite(device.mit_kd) || device.mit_kd < 0.0 ||
        device.mit_kd > device.limits.kd_max) {
      throw std::invalid_argument(device.name +
                                  " mit_kd is outside model limits");
    }
    if (!std::isfinite(device.torque_constant_nm_per_arms) ||
        device.torque_constant_nm_per_arms <= 0.0) {
      throw std::invalid_argument(device.name +
                                  " torque constant must be positive");
    }
    if (!std::isfinite(device.startup_preload_max) ||
        device.startup_preload_max < 0.0) {
      throw std::invalid_argument(device.name +
                                  " startup preload limit is invalid");
    }
    devices_.push_back(device);
  }
  reset_state();
}

void RobStrideProtocol::reset_state() {
  enable_check_started_ = false;
  enable_check_start_ = {};
  for (auto &device : devices_) {
    device.position = 0.0;
    device.velocity = 0.0;
    device.effort = 0.0;
    device.temperature_c = 0.0;
    device.command_position = std::numeric_limits<double>::quiet_NaN();
    device.command_velocity = 0.0;
    device.command_t_ff = 0.0;
    device.command_kp = device.mit_kp;
    device.command_kd = device.mit_kd;
    device.filtered_current_peak = 0.0;
    device.run_mode = 0;
    device.mode_state = 0;
    device.position_valid = false;
    device.status_received = false;
    device.mechanical_position_received = false;
    device.filtered_current_received = false;
    device.run_mode_received = false;
    device.faulted = false;
    device.last_status = {};
    device.stable_since = {};
    device.stable_samples = 0;
    device.startup_moved = false;
  }
}

std::vector<shared_can_interface::v1::CanFrame>
RobStrideProtocol::initial_state_requests() const {
  std::vector<shared_can_interface::v1::CanFrame> frames;
  frames.reserve(devices_.size() * 3U);
  for (const auto &device : devices_) {
    frames.push_back(codec::make_read_parameter(device.motor_id, host_id_,
                                                kMechanicalPosition));
    frames.push_back(
        codec::make_read_parameter(device.motor_id, host_id_, kRunMode));
    frames.push_back(codec::make_read_parameter(device.motor_id, host_id_,
                                                kFilteredCurrent));
  }
  return frames;
}

bool RobStrideProtocol::initial_state_ready() const {
  return !devices_.empty() &&
         std::all_of(devices_.begin(), devices_.end(),
                     [](const Device &device) {
                       return device.mechanical_position_received &&
                              device.filtered_current_received &&
                              device.run_mode_received;
                     });
}

std::vector<std::string> RobStrideProtocol::missing_initial_states() const {
  std::vector<std::string> names;
  for (const auto &device : devices_) {
    if (!device.mechanical_position_received) {
      names.push_back(device.name);
    }
    if (!device.run_mode_received) {
      names.push_back(device.name + "/run_mode");
    }
    if (!device.filtered_current_received) {
      names.push_back(device.name + "/iqf");
    }
  }
  return names;
}

bool RobStrideProtocol::process_frame(
    const shared_can_interface::v1::CanFrame &frame,
    std::chrono::steady_clock::time_point timestamp) {
  if (!frame.extended) {
    return false;
  }
  const auto id = codec::parse_extended_id(frame.arbitration_id);
  if (id.node_id != host_id_) {
    return false;
  }

  if (const auto parameter = codec::decode_parameter_response(frame)) {
    auto *device = find_by_motor_id(parameter->device_id);
    if (device == nullptr) {
      return false;
    }
    if (parameter->parameter_id == kMechanicalPosition) {
      const double motor_position = codec::parameter_as_f32(*parameter);
      if (!std::isfinite(motor_position) ||
          std::abs(motor_position) > device->limits.position_max) {
        return false;
      }
      // 0x7019 is used exactly once to establish a safe hold target before
      // enabling the motor. Runtime state is never overwritten by parameter
      // responses; operation-status is the only running feedback source.
      if (!device->mechanical_position_received) {
        device->position = device->direction * motor_position + device->offset;
        device->position_valid = true;
        device->command_position = device->position;
        device->mechanical_position_received = true;
      }
    } else if (parameter->parameter_id == kRunMode) {
      device->run_mode = parameter->raw_value[0];
      device->run_mode_received = true;
    } else if (parameter->parameter_id == kFilteredCurrent) {
      const double current = codec::parameter_as_f32(*parameter);
      if (!std::isfinite(current)) {
        return false;
      }
      device->filtered_current_peak = current;
      device->filtered_current_received = true;
    }
    return true;
  }

  if (id.communication_type != codec::CommunicationType::kOperationStatus) {
    return false;
  }
  auto *device =
      find_by_motor_id(static_cast<std::uint8_t>(id.extra_data & 0xFFU));
  if (device == nullptr) {
    return false;
  }
  const auto status = codec::decode_status(frame, device->limits);
  if (!status) {
    return false;
  }
  if (device->status_received && timestamp <= device->last_status) {
    return true;
  }
  device->position = device->direction * status->position + device->offset;
  device->velocity = device->direction * status->velocity;
  device->effort = device->direction * status->torque;
  device->temperature_c = status->temperature_c;
  device->mode_state = status->mode_state;
  device->position_valid = true;
  device->status_received = true;
  device->faulted = status->uncalibrated || status->stall ||
                    status->magnetic_encoder_fault || status->overtemperature ||
                    status->overcurrent || status->undervoltage;
  if (enable_check_started_ && timestamp >= enable_check_start_ &&
      timestamp > device->last_status && device->run_mode == 0U) {
    if (device->mode_state == 0U &&
        std::abs(device->position - device->check_initial_position) >
            kStartupMaximumDisplacement) {
      device->startup_moved = true;
    }
    // Empirical guard for the observed RS firmware startup placeholder, not
    // a manufacturer-defined ready bit. An exact 0-C cold motor is ambiguous
    // and deliberately cannot pass this gate; do not auto-reset it.
    const bool stable_candidate = device->mode_state == 0U &&
                                  !device->faulted && !device->startup_moved &&
                                  device->temperature_c > 0.0 &&
                                  std::abs(device->velocity) <= 0.5;
    if (!stable_candidate) {
      device->stable_samples = 0;
    } else {
      const double q_min =
          std::min(device->stable_position_min, device->position);
      const double q_max =
          std::max(device->stable_position_max, device->position);
      const double t_min =
          std::min(device->stable_temperature_min, device->temperature_c);
      const double t_max =
          std::max(device->stable_temperature_max, device->temperature_c);
      if (device->stable_samples == 0 ||
          timestamp - device->last_status > kStartupFeedbackFreshness ||
          q_max - q_min > kStartupStablePositionSpan || t_max - t_min > 1.0) {
        device->stable_since = timestamp;
        device->stable_samples = 1;
        device->stable_position_min = device->stable_position_max =
            device->position;
        device->stable_temperature_min = device->stable_temperature_max =
            device->temperature_c;
      } else {
        device->stable_samples =
            std::min<std::size_t>(device->stable_samples + 1, 3);
        device->stable_position_min = q_min;
        device->stable_position_max = q_max;
        device->stable_temperature_min = t_min;
        device->stable_temperature_max = t_max;
      }
    }
  }
  device->last_status = timestamp;
  return true;
}

std::optional<shared_can_interface::v1::CanFrame>
RobStrideProtocol::periodic_request() {
  return std::nullopt;
}

void RobStrideProtocol::begin_enable_check(
    std::chrono::steady_clock::time_point now) {
  enable_check_started_ = true;
  enable_check_start_ = now;
  for (auto &device : devices_) {
    device.status_received = false;
    device.last_status = {};
    device.stable_samples = 0;
    device.startup_moved = false;
    device.check_initial_position = device.position;
  }
}

std::vector<std::string> RobStrideProtocol::pending_enable_states(
    std::chrono::steady_clock::time_point now) const {
  std::vector<std::string> pending;
  for (const auto &device : devices_) {
    // Non-MIT modes retain their existing mode-switch sequence. The repeated
    // MIT-probe readiness guard is only validated for motors already in MIT.
    if (device.run_mode_received && device.run_mode != 0U) {
      continue;
    }
    std::string reason;
    if (!enable_check_started_ || !device.run_mode_received ||
        !device.status_received || device.last_status < enable_check_start_ ||
        now < device.last_status ||
        now - device.last_status > kStartupFeedbackFreshness) {
      reason = "missing fresh MIT status";
    } else if (device.faulted) {
      reason = "motor fault";
    } else if (device.startup_moved) {
      reason = "moved more than 0.025 rad while disabled; support and retry "
               "activation";
    } else if (device.mode_state != 0U && device.mode_state != 2U) {
      reason = "unexpected motor state " + std::to_string(device.mode_state);
    } else if (device.temperature_c <= 0.0) {
      reason = "ambiguous 0-C startup feedback (not a firmware ready flag)";
    } else if (device.mode_state == 0U &&
               (device.stable_samples < 3 ||
                device.last_status - device.stable_since <
                    kStartupStableDuration)) {
      reason = "waiting for three stable disabled replies over at least 30 ms";
    }
    if (!reason.empty()) {
      pending.push_back(device.name + ": " + reason);
    }
  }
  return pending;
}

std::vector<shared_can_interface::v1::TimedFrameBatch>
RobStrideProtocol::pre_enable_sequence() const {
  shared_can_interface::v1::TimedFrameBatch probe;
  probe.delay_after = std::chrono::milliseconds(10);
  probe.frames.reserve(devices_.size());
  for (const auto &device : devices_) {
    if (!device.run_mode_received || device.run_mode != 0U) {
      continue;
    }
    if (!std::isfinite(device.command_position)) {
      throw std::runtime_error("cannot probe RobStride device without command "
                               "position: " +
                               device.name);
    }
    const double probe_position = enable_check_started_ &&
                                          device.status_received &&
                                          device.mode_state == 0U
                                      ? device.position
                                      : device.command_position;
    const double motor_target =
        device.direction * (probe_position - device.offset);
    probe.frames.push_back(codec::make_mit_command(
        device.motor_id, device.limits, static_cast<float>(motor_target), 0.0F,
        static_cast<float>(device.command_kp),
        static_cast<float>(device.command_kd), 0.0F));
  }
  if (probe.frames.empty()) {
    return {};
  }
  return {std::move(probe)};
}

std::vector<shared_can_interface::v1::TimedFrameBatch>
RobStrideProtocol::enable_sequence() const {
  if (enable_check_started_) {
    const auto pending =
        pending_enable_states(std::chrono::steady_clock::now());
    if (!pending.empty()) {
      throw std::runtime_error("cannot enable RobStride: " + pending.front());
    }
  }
  shared_can_interface::v1::TimedFrameBatch set_mode;
  shared_can_interface::v1::TimedFrameBatch enable_and_hold;

  for (const auto &device : devices_) {
    if (!device.position_valid) {
      throw std::runtime_error(
          "cannot enable RobStride device without position: " + device.name);
    }
    if (!device.run_mode_received) {
      throw std::runtime_error("cannot enable RobStride device without run "
                               "mode: " +
                               device.name);
    }
    if (device.run_mode != 0U) {
      set_mode.frames.push_back(codec::make_write_parameter_i8(
          device.motor_id, host_id_, kRunMode, 0));
    }
    if (!std::isfinite(device.command_position)) {
      throw std::runtime_error("cannot enable RobStride device without command "
                               "position: " +
                               device.name);
    }
    const double motor_target =
        device.direction * (device.command_position - device.offset);
    // A redundant enable can briefly reset an already-running motor's hold
    // controller. pre_enable_sequence() has just refreshed the hold command
    // and collected its status, so only enable a motor that is not already in
    // Motor mode (mode_state == 2).
    const bool already_running_in_mit = device.run_mode == 0U &&
                                        device.status_received &&
                                        device.mode_state == 2U;
    if (!already_running_in_mit) {
      enable_and_hold.frames.push_back(
          codec::make_enable(device.motor_id, host_id_));
    }
    // Keep the enable-to-hold gap to one CAN frame for a genuinely disabled
    // motor. Do not insert a sleep here: gravity can move an arm before the
    // first MIT command arrives.
    enable_and_hold.frames.push_back(codec::make_mit_command(
        device.motor_id, device.limits, static_cast<float>(motor_target), 0.0F,
        static_cast<float>(device.command_kp),
        static_cast<float>(device.command_kd), 0.0F));
  }

  std::vector<shared_can_interface::v1::TimedFrameBatch> sequence;
  if (!set_mode.frames.empty()) {
    set_mode.delay_after = std::chrono::milliseconds(40);
    sequence.push_back(std::move(set_mode));
  }
  sequence.push_back(std::move(enable_and_hold));
  return sequence;
}

std::vector<shared_can_interface::v1::CanFrame>
RobStrideProtocol::disable_frames() const {
  std::vector<shared_can_interface::v1::CanFrame> frames;
  frames.reserve(devices_.size());
  for (const auto &device : devices_) {
    frames.push_back(codec::make_disable(device.motor_id, host_id_));
  }
  return frames;
}

bool RobStrideProtocol::set_command(const std::string &device_name,
                                    const std::string &interface_name,
                                    double value) {
  auto *device = find_by_name(device_name);
  if (device == nullptr || !std::isfinite(value)) {
    return false;
  }
  if (interface_name == "position") {
    device->command_position = value;
    return true;
  }
  if (interface_name == "velocity") {
    device->command_velocity = value;
    return true;
  }
  if (interface_name == "t_ff") {
    device->command_t_ff = value;
    return true;
  }
  if (interface_name == "kp") {
    device->command_kp = value;
    return true;
  }
  if (interface_name == "kd") {
    device->command_kd = value;
    return true;
  }
  return false;
}

std::size_t RobStrideProtocol::command_frame_capacity() const noexcept {
  return devices_.size();
}

void RobStrideProtocol::command_frames(
    std::vector<shared_can_interface::v1::CanFrame> &output) {
  output.clear();
  if (output.capacity() < devices_.size()) {
    output.reserve(devices_.size());
  }
  for (const auto &device : devices_) {
    if (!std::isfinite(device.command_position)) {
      continue;
    }
    const double motor_target =
        device.direction * (device.command_position - device.offset);
    const double motor_velocity = device.direction * device.command_velocity;
    const double motor_t_ff = device.direction * device.command_t_ff;
    output.push_back(codec::make_mit_command(
        device.motor_id, device.limits, static_cast<float>(motor_target),
        static_cast<float>(motor_velocity),
        static_cast<float>(device.command_kp),
        static_cast<float>(device.command_kd), static_cast<float>(motor_t_ff)));
  }
}

bool RobStrideProtocol::get_state(const std::string &device_name,
                                  const std::string &interface_name,
                                  double &value) const {
  const auto *device = find_by_name(device_name);
  if (device == nullptr) {
    return false;
  }
  if (interface_name == "position" && device->position_valid) {
    value = device->position;
    return true;
  }
  if (interface_name == "velocity" && device->status_received) {
    value = device->velocity;
    return true;
  }
  if (interface_name == "effort" && device->status_received) {
    value = device->effort;
    return true;
  }
  if (interface_name == "temperature" && device->status_received) {
    value = device->temperature_c;
    return true;
  }
  return false;
}

bool RobStrideProtocol::get_initial_command(const std::string &device_name,
                                            const std::string &interface_name,
                                            double &value) const {
  const auto *device = find_by_name(device_name);
  if (device == nullptr) {
    return false;
  }
  if (interface_name == "position" && enable_check_started_ &&
      device->status_received && device->run_mode == 0U) {
    // Reseeding after the wait must not erase an already-issued warm preload.
    // A disabled motor instead starts at the latest measured, supported pose.
    value =
        device->mode_state == 2U ? device->command_position : device->position;
    return std::isfinite(value);
  }
  if (interface_name != "position" || !device->position_valid ||
      !device->filtered_current_received || !device->run_mode_received ||
      device->run_mode != 0U || device->mit_kp <= 0.0) {
    return get_state(device_name, interface_name, value);
  }

  const double joint_torque = device->direction *
                              device->filtered_current_peak * kPeakToRms *
                              device->torque_constant_nm_per_arms;
  const double preload =
      std::clamp(joint_torque / device->mit_kp, -device->startup_preload_max,
                 device->startup_preload_max);
  value = device->position + preload;
  return std::isfinite(value);
}

bool RobStrideProtocol::faulted(const std::string &device_name) const {
  const auto *device = find_by_name(device_name);
  return device != nullptr && device->faulted;
}

RobStrideProtocol::Device *
RobStrideProtocol::find_by_motor_id(std::uint8_t motor_id) {
  const auto iterator = std::find_if(
      devices_.begin(), devices_.end(),
      [motor_id](const Device &device) { return device.motor_id == motor_id; });
  return iterator == devices_.end() ? nullptr : &*iterator;
}

const RobStrideProtocol::Device *
RobStrideProtocol::find_by_name(const std::string &name) const {
  const auto iterator = std::find_if(
      devices_.begin(), devices_.end(),
      [&name](const Device &device) { return device.name == name; });
  return iterator == devices_.end() ? nullptr : &*iterator;
}

RobStrideProtocol::Device *
RobStrideProtocol::find_by_name(const std::string &name) {
  const auto iterator = std::find_if(
      devices_.begin(), devices_.end(),
      [&name](const Device &device) { return device.name == name; });
  return iterator == devices_.end() ? nullptr : &*iterator;
}

} // namespace rebot_robstride_protocol

PLUGINLIB_EXPORT_CLASS(rebot_robstride_protocol::RobStrideProtocol,
                       shared_can_interface::v1::ProtocolInterface)
