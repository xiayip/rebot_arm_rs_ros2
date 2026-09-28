#include "rebot_robstride_protocol/rebot_gripper_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

#include "pluginlib/class_list_macros.hpp"

namespace rebot_robstride_protocol
{
namespace
{

const std::string & required_parameter(
  const shared_can_interface::v1::ParameterMap & parameters, const std::string & name)
{
  const auto item = parameters.find(name);
  if (item == parameters.end() || item->second.empty()) {
    throw std::invalid_argument("missing parameter: " + name);
  }
  return item->second;
}

double optional_parameter(
  const shared_can_interface::v1::ParameterMap & parameters, const std::string & name,
  double fallback)
{
  const auto item = parameters.find(name);
  return item == parameters.end() ? fallback : std::stod(item->second);
}

}  // namespace

void RebotGripperProtocol::configure(
  const shared_can_interface::v1::ParameterMap & bus_parameters,
  const std::vector<shared_can_interface::v1::DeviceConfiguration> & devices)
{
  if (devices.empty()) {
    throw std::invalid_argument("RebotGripper protocol requires at least one device");
  }

  std::vector<shared_can_interface::v1::DeviceConfiguration> motor_devices;
  motor_devices.reserve(devices.size());
  devices_.clear();
  devices_.reserve(devices.size());
  std::set<std::string> names;
  for (const auto & configuration : devices) {
    if (!names.insert(configuration.name).second) {
      throw std::invalid_argument("duplicate RebotGripper device: " + configuration.name);
    }
    for (const auto & interface : configuration.command_interfaces) {
      if (interface != "position" && interface != "velocity") {
        throw std::invalid_argument(
                configuration.name + " has unsupported gripper command interface: " + interface);
      }
    }
    if (std::find(
        configuration.command_interfaces.begin(), configuration.command_interfaces.end(),
        "position") == configuration.command_interfaces.end())
    {
      throw std::invalid_argument(configuration.name + " requires a position command interface");
    }
    for (const auto & interface : configuration.state_interfaces) {
      if (interface != "position" && interface != "velocity" && interface != "effort" &&
        interface != "temperature")
      {
        throw std::invalid_argument(
                configuration.name + " has unsupported gripper state interface: " + interface);
      }
    }

    Device device;
    device.name = configuration.name;
    device.open_position =
      std::stod(required_parameter(configuration.parameters, "open_position"));
    device.closed_position =
      std::stod(required_parameter(configuration.parameters, "closed_position"));
    device.default_velocity =
      optional_parameter(configuration.parameters, "default_velocity", 0.25);
    if (!std::isfinite(device.open_position) || !std::isfinite(device.closed_position) ||
      std::abs(device.open_position - device.closed_position) < 1e-9)
    {
      throw std::invalid_argument(
              configuration.name + " open_position and closed_position must be finite and distinct");
    }
    if (!std::isfinite(device.default_velocity) || device.default_velocity < 0.0 ||
      device.default_velocity > 1.0)
    {
      throw std::invalid_argument(configuration.name + " default_velocity must be within [0, 1]");
    }
    const auto limits = codec::limits_for_model(
      required_parameter(configuration.parameters, "model"));
    if (std::abs(device.open_position) > limits.position_max ||
      std::abs(device.closed_position) > limits.position_max)
    {
      throw std::invalid_argument(configuration.name + " gripper endpoint exceeds motor limits");
    }
    device.velocity_limit = device.default_velocity;
    devices_.push_back(device);

    auto motor_configuration = configuration;
    motor_configuration.command_interfaces = {"position"};
    motor_devices.push_back(std::move(motor_configuration));
  }
  motor_protocol_.configure(bus_parameters, motor_devices);
  reset_state();
}

void RebotGripperProtocol::reset_state()
{
  motor_protocol_.reset_state();
  for (auto & device : devices_) {
    device.target_position = std::numeric_limits<double>::quiet_NaN();
    device.sent_position = std::numeric_limits<double>::quiet_NaN();
    device.velocity_limit = device.default_velocity;
    device.last_command_time = {};
  }
}

std::vector<shared_can_interface::v1::CanFrame>
RebotGripperProtocol::initial_state_requests() const
{
  return motor_protocol_.initial_state_requests();
}

bool RebotGripperProtocol::initial_state_ready() const
{
  return motor_protocol_.initial_state_ready();
}

std::vector<std::string> RebotGripperProtocol::missing_initial_states() const
{
  return motor_protocol_.missing_initial_states();
}

bool RebotGripperProtocol::process_frame(
  const shared_can_interface::v1::CanFrame & frame,
  std::chrono::steady_clock::time_point timestamp)
{
  return motor_protocol_.process_frame(frame, timestamp);
}

std::optional<shared_can_interface::v1::CanFrame> RebotGripperProtocol::periodic_request()
{
  return motor_protocol_.periodic_request();
}

std::vector<shared_can_interface::v1::TimedFrameBatch>
RebotGripperProtocol::pre_enable_sequence() const
{
  return motor_protocol_.pre_enable_sequence();
}

void RebotGripperProtocol::begin_enable_check(std::chrono::steady_clock::time_point now)
{
  motor_protocol_.begin_enable_check(now);
}

std::vector<std::string> RebotGripperProtocol::pending_enable_states(
  std::chrono::steady_clock::time_point now) const
{
  return motor_protocol_.pending_enable_states(now);
}

std::vector<shared_can_interface::v1::TimedFrameBatch>
RebotGripperProtocol::enable_sequence() const
{
  return motor_protocol_.enable_sequence();
}

std::vector<shared_can_interface::v1::CanFrame> RebotGripperProtocol::disable_frames() const
{
  return motor_protocol_.disable_frames();
}

bool RebotGripperProtocol::set_command(
  const std::string & device_name, const std::string & interface_name, double value)
{
  auto * device = find(device_name);
  if (device == nullptr || !std::isfinite(value)) {
    return false;
  }
  if (interface_name == "position") {
    if (value < 0.0 || value > 1.0) {
      return false;
    }
    device->target_position = value;
    return true;
  }
  if (interface_name == "velocity") {
    if (value < 0.0 || value > 1.0) {
      return false;
    }
    device->velocity_limit = value;
    return true;
  }
  return false;
}

std::size_t RebotGripperProtocol::command_frame_capacity() const noexcept
{
  return motor_protocol_.command_frame_capacity();
}

void RebotGripperProtocol::command_frames(
  std::vector<shared_can_interface::v1::CanFrame> & output)
{
  const auto now = std::chrono::steady_clock::now();
  for (auto & device : devices_) {
    if (!std::isfinite(device.target_position)) {
      continue;
    }
    if (!std::isfinite(device.sent_position)) {
      double motor_position = 0.0;
      if (!motor_protocol_.get_state(device.name, "position", motor_position)) {
        continue;
      }
      device.sent_position = to_normalized(device, motor_position);
      device.last_command_time = now;
    }
    const double elapsed = std::max(
      0.0, std::chrono::duration<double>(now - device.last_command_time).count());
    const double maximum_step = device.velocity_limit * elapsed;
    const double error = device.target_position - device.sent_position;
    device.sent_position += std::clamp(error, -maximum_step, maximum_step);
    device.sent_position = std::clamp(device.sent_position, 0.0, 1.0);
    device.last_command_time = now;
    motor_protocol_.set_command(
      device.name, "position", to_motor(device, device.sent_position));
  }
  motor_protocol_.command_frames(output);
}

bool RebotGripperProtocol::get_state(
  const std::string & device_name, const std::string & interface_name, double & value) const
{
  const auto * device = find(device_name);
  if (device == nullptr) {
    return false;
  }
  if (interface_name == "position") {
    double motor_position = 0.0;
    if (!motor_protocol_.get_state(device_name, interface_name, motor_position)) {
      return false;
    }
    value = to_normalized(*device, motor_position);
    return true;
  }
  if (interface_name == "velocity") {
    double motor_velocity = 0.0;
    if (!motor_protocol_.get_state(device_name, interface_name, motor_velocity)) {
      return false;
    }
    value = -motor_velocity / (device->open_position - device->closed_position);
    return std::isfinite(value);
  }
  return motor_protocol_.get_state(device_name, interface_name, value);
}

bool RebotGripperProtocol::get_initial_command(
  const std::string & device_name, const std::string & interface_name, double & value) const
{
  const auto * device = find(device_name);
  if (device == nullptr) {
    return false;
  }
  if (interface_name == "position") {
    double motor_position = 0.0;
    if (!motor_protocol_.get_initial_command(device_name, interface_name, motor_position)) {
      return false;
    }
    value = to_normalized(*device, motor_position);
    return true;
  }
  if (interface_name == "velocity") {
    value = device->velocity_limit;
    return true;
  }
  return false;
}

bool RebotGripperProtocol::faulted(const std::string & device_name) const
{
  return motor_protocol_.faulted(device_name);
}

RebotGripperProtocol::Device * RebotGripperProtocol::find(const std::string & name)
{
  const auto item = std::find_if(
    devices_.begin(), devices_.end(),
    [&name](const Device & device) {return device.name == name;});
  return item == devices_.end() ? nullptr : &*item;
}

const RebotGripperProtocol::Device * RebotGripperProtocol::find(
  const std::string & name) const
{
  const auto item = std::find_if(
    devices_.begin(), devices_.end(),
    [&name](const Device & device) {return device.name == name;});
  return item == devices_.end() ? nullptr : &*item;
}

double RebotGripperProtocol::to_normalized(
  const Device & device, double motor_position) const
{
  return std::clamp(
    (device.open_position - motor_position) /
    (device.open_position - device.closed_position),
    0.0, 1.0);
}

double RebotGripperProtocol::to_motor(
  const Device & device, double normalized_position) const
{
  return device.open_position - normalized_position *
         (device.open_position - device.closed_position);
}

}  // namespace rebot_robstride_protocol

PLUGINLIB_EXPORT_CLASS(
  rebot_robstride_protocol::RebotGripperProtocol,
  shared_can_interface::v1::ProtocolInterface)
