#include "rebot_robstride_protocol/robstride_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace rebot_robstride_protocol {
namespace {

constexpr std::uint16_t kRunMode = 0x7005;
constexpr std::uint16_t kMechanicalPosition = 0x7019;

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
      if (interface_name != "position") {
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
    device.limits = codec::limits_for_model(
        required_parameter(configuration.parameters, "model"));
    device.direction =
        std::stod(required_parameter(configuration.parameters, "direction"));
    device.offset =
        std::stod(required_parameter(configuration.parameters, "offset"));
    device.kp =
        std::stod(required_parameter(configuration.parameters, "position_kp"));
    device.kd =
        std::stod(required_parameter(configuration.parameters, "velocity_kd"));
    if (std::abs(std::abs(device.direction) - 1.0) > 1e-9) {
      throw std::invalid_argument(device.name + " direction must be +1 or -1");
    }
    devices_.push_back(device);
  }
  next_poll_device_ = 0;
  reset_state();
}

void RobStrideProtocol::reset_state() {
  for (auto &device : devices_) {
    device.position = 0.0;
    device.velocity = 0.0;
    device.effort = 0.0;
    device.temperature_c = 0.0;
    device.command_position = std::numeric_limits<double>::quiet_NaN();
    device.position_valid = false;
    device.status_received = false;
    device.mechanical_position_received = false;
    device.faulted = false;
    device.last_mechanical_position = 0.0;
    device.last_mechanical_position_time = {};
  }
}

std::vector<shared_can_interface::v1::CanFrame>
RobStrideProtocol::initial_state_requests() const {
  std::vector<shared_can_interface::v1::CanFrame> frames;
  frames.reserve(devices_.size());
  for (const auto &device : devices_) {
    frames.push_back(codec::make_read_parameter(device.motor_id, host_id_,
                                                kMechanicalPosition));
  }
  return frames;
}

bool RobStrideProtocol::initial_state_ready() const {
  return !devices_.empty() &&
         std::all_of(devices_.begin(), devices_.end(),
                     [](const Device &device) {
                       return device.mechanical_position_received;
                     });
}

std::vector<std::string> RobStrideProtocol::missing_initial_states() const {
  std::vector<std::string> names;
  for (const auto &device : devices_) {
    if (!device.mechanical_position_received) {
      names.push_back(device.name);
    }
  }
  return names;
}

bool RobStrideProtocol::update_mechanical_position(
    Device &device, double motor_position,
    std::chrono::steady_clock::time_point timestamp) {
  const double next_position =
      device.direction * motor_position + device.offset;
  if (device.mechanical_position_received &&
      timestamp > device.last_mechanical_position_time) {
    const double seconds = std::chrono::duration<double>(
                               timestamp - device.last_mechanical_position_time)
                               .count();
    if (seconds > 1e-6) {
      const double measured_velocity =
          (next_position - device.last_mechanical_position) / seconds;
      if (std::abs(measured_velocity) <= device.limits.velocity_max) {
        device.velocity = measured_velocity;
      } else {
        return false;
      }
    }
  }
  device.position = next_position;
  device.position_valid = true;
  device.last_mechanical_position = next_position;
  device.last_mechanical_position_time = timestamp;
  return true;
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
      if (!update_mechanical_position(*device, motor_position, timestamp)) {
        return false;
      }
      if (!std::isfinite(device->command_position)) {
        device->command_position = device->position;
      }
      device->mechanical_position_received = true;
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
  device->effort = device->direction * status->torque;
  device->temperature_c = status->temperature_c;
  device->status_received = true;
  device->faulted = status->uncalibrated || status->stall ||
                    status->magnetic_encoder_fault || status->overtemperature ||
                    status->overcurrent || status->undervoltage;
  return true;
}

std::optional<shared_can_interface::v1::CanFrame>
RobStrideProtocol::periodic_request() {
  if (devices_.empty()) {
    return std::nullopt;
  }
  const auto &device = devices_[next_poll_device_];
  next_poll_device_ = (next_poll_device_ + 1U) % devices_.size();
  return codec::make_read_parameter(device.motor_id, host_id_,
                                    kMechanicalPosition);
}

std::vector<shared_can_interface::v1::TimedFrameBatch>
RobStrideProtocol::enable_sequence() const {
  shared_can_interface::v1::TimedFrameBatch set_mode;
  set_mode.delay_after = std::chrono::milliseconds(40);
  shared_can_interface::v1::TimedFrameBatch enable;
  enable.delay_after = std::chrono::milliseconds(10);
  shared_can_interface::v1::TimedFrameBatch hold_position;

  for (const auto &device : devices_) {
    if (!device.position_valid) {
      throw std::runtime_error(
          "cannot enable RobStride device without position: " + device.name);
    }
    set_mode.frames.push_back(
        codec::make_write_parameter_i8(device.motor_id, host_id_, kRunMode, 0));
    enable.frames.push_back(codec::make_enable(device.motor_id, host_id_));
    const double motor_target =
        device.direction * (device.position - device.offset);
    hold_position.frames.push_back(codec::make_mit_command(
        device.motor_id, device.limits, static_cast<float>(motor_target), 0.0F,
        static_cast<float>(device.kp), static_cast<float>(device.kd), 0.0F));
  }
  return {std::move(set_mode), std::move(enable), std::move(hold_position)};
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
  if (device == nullptr || interface_name != "position" ||
      !std::isfinite(value)) {
    return false;
  }
  device->command_position = value;
  return true;
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
    output.push_back(codec::make_mit_command(
        device.motor_id, device.limits, static_cast<float>(motor_target), 0.0F,
        static_cast<float>(device.kp), static_cast<float>(device.kd), 0.0F));
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
  if (interface_name == "velocity" && device->position_valid) {
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
