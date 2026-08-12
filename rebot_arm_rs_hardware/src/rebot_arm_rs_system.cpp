#include "rebot_arm_rs_hardware/rebot_arm_rs_system.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <set>
#include <stdexcept>
#include <thread>
#include <unordered_map>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rebot_arm_rs_hardware {
namespace {

constexpr std::uint16_t kRunMode = 0x7005;
constexpr std::uint16_t kMechanicalPosition = 0x7019;

bool parse_bool(const std::string &value) {
  return value == "true" || value == "1" || value == "yes";
}

std::uint8_t parse_u8(const std::string &value) {
  const auto parsed = std::stoul(value, nullptr, 0);
  if (parsed > 0xFFU) {
    throw std::out_of_range("8-bit value out of range: " + value);
  }
  return static_cast<std::uint8_t>(parsed);
}

const std::string &required_parameter(const std::unordered_map<std::string, std::string> &parameters,
                                      const std::string &name) {
  const auto iterator = parameters.find(name);
  if (iterator == parameters.end() || iterator->second.empty()) {
    throw std::invalid_argument("missing parameter: " + name);
  }
  return iterator->second;
}

}  // namespace

RebotArmRsSystem::CallbackReturn RebotArmRsSystem::on_init(
    const hardware_interface::HardwareComponentInterfaceParams &params) {
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    const auto &hardware = info_.hardware_parameters;
    can_interface_ = required_parameter(hardware, "can_interface");
    host_id_ = parse_u8(required_parameter(hardware, "host_id"));
    if (const auto it = hardware.find("auto_enable_on_activate"); it != hardware.end()) {
      auto_enable_on_activate_ = parse_bool(it->second);
    }
    if (const auto it = hardware.find("require_all_motors"); it != hardware.end()) {
      require_all_motors_ = parse_bool(it->second);
    }
    if (const auto it = hardware.find("disable_on_deactivate"); it != hardware.end()) {
      disable_on_deactivate_ = parse_bool(it->second);
    }
    if (const auto it = hardware.find("initial_read_timeout_ms"); it != hardware.end()) {
      initial_read_timeout_ms_ = std::max(100, std::stoi(it->second));
    }
    if (const auto it = hardware.find("position_poll_divider"); it != hardware.end()) {
      position_poll_divider_ = std::max<std::size_t>(1, std::stoul(it->second));
    }
    if (const auto it = hardware.find("max_command_step"); it != hardware.end()) {
      max_command_step_ = std::max(0.001, std::stod(it->second));
    }

    if (info_.joints.size() != 6U) {
      throw std::invalid_argument("B601-RS without gripper requires exactly six joints");
    }

    std::set<std::uint8_t> motor_ids;
    joints_.reserve(info_.joints.size());
    for (const auto &joint_info : info_.joints) {
      if (joint_info.command_interfaces.size() != 1U ||
          joint_info.command_interfaces.front().name != hardware_interface::HW_IF_POSITION) {
        throw std::invalid_argument(joint_info.name + " must expose one position command interface");
      }
      std::set<std::string> state_names;
      for (const auto &state : joint_info.state_interfaces) {
        state_names.insert(state.name);
      }
      if (state_names != std::set<std::string>{hardware_interface::HW_IF_POSITION,
                                               hardware_interface::HW_IF_VELOCITY,
                                               hardware_interface::HW_IF_EFFORT}) {
        throw std::invalid_argument(joint_info.name + " must expose position, velocity, and effort states");
      }

      Joint joint;
      joint.name = joint_info.name;
      joint.motor_id = parse_u8(required_parameter(joint_info.parameters, "motor_id"));
      joint.model = required_parameter(joint_info.parameters, "model");
      joint.motor_limits = robstride::limits_for_model(joint.model);
      joint.direction = std::stod(required_parameter(joint_info.parameters, "direction"));
      joint.offset = std::stod(required_parameter(joint_info.parameters, "offset"));
      joint.kp = std::stod(required_parameter(joint_info.parameters, "position_kp"));
      joint.kd = std::stod(required_parameter(joint_info.parameters, "velocity_kd"));
      joint.min_position = std::stod(required_parameter(joint_info.parameters, "min_position"));
      joint.max_position = std::stod(required_parameter(joint_info.parameters, "max_position"));
      if (std::abs(std::abs(joint.direction) - 1.0) > 1e-9) {
        throw std::invalid_argument(joint.name + " direction must be +1 or -1");
      }
      if (!motor_ids.insert(joint.motor_id).second) {
        throw std::invalid_argument("duplicate motor ID");
      }
      joints_.push_back(joint);
    }
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "B601-RS initialization failed: %s", error.what());
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Configured B601-RS on %s with host ID 0x%02X; auto-enable=%s",
              can_interface_.c_str(), host_id_, auto_enable_on_activate_ ? "true" : "false");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface::ConstSharedPtr>
RebotArmRsSystem::on_export_state_interfaces() {
  auto interfaces = hardware_interface::SystemInterface::on_export_state_interfaces();
  position_state_handles_.clear();
  velocity_state_handles_.clear();
  effort_state_handles_.clear();
  for (const auto &joint : joints_) {
    position_state_handles_.push_back(get_state_interface_handle(joint.name + "/position"));
    velocity_state_handles_.push_back(get_state_interface_handle(joint.name + "/velocity"));
    effort_state_handles_.push_back(get_state_interface_handle(joint.name + "/effort"));
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface::SharedPtr>
RebotArmRsSystem::on_export_command_interfaces() {
  auto interfaces = hardware_interface::SystemInterface::on_export_command_interfaces();
  position_command_handles_.clear();
  for (const auto &joint : joints_) {
    position_command_handles_.push_back(get_command_interface_handle(joint.name + "/position"));
  }
  return interfaces;
}

RebotArmRsSystem::CallbackReturn RebotArmRsSystem::on_configure(
    const rclcpp_lifecycle::State &) {
  try {
    transport_.open(can_interface_);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "Unable to open SocketCAN %s: %s", can_interface_.c_str(), error.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(get_logger(), "SocketCAN %s opened", can_interface_.c_str());
  return CallbackReturn::SUCCESS;
}

bool RebotArmRsSystem::read_initial_positions() {
  drain_receive_queue(256);
  for (auto &joint : joints_) {
    joint.has_position = false;
    joint.mechanical_position_received = false;
    joint.velocity = 0.0;
    transport_.send(robstride::make_read_parameter(joint.motor_id, host_id_, kMechanicalPosition));
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(initial_read_timeout_ms_);
  while (std::chrono::steady_clock::now() < deadline) {
    if (const auto frame = transport_.receive(std::chrono::milliseconds(20))) {
      process_frame(*frame);
    }
    if (std::all_of(joints_.begin(), joints_.end(), [](const Joint &joint) {
          return joint.mechanical_position_received;
        })) {
      break;
    }
  }

  bool all_received = true;
  for (auto &joint : joints_) {
    if (!joint.mechanical_position_received) {
      all_received = false;
      RCLCPP_ERROR(get_logger(), "No initial position from %s (motor 0x%02X)", joint.name.c_str(),
                   joint.motor_id);
      continue;
    }
    joint.last_sent_position = joint.position;
    RCLCPP_INFO(get_logger(), "Initial %s (motor 0x%02X): %.6f rad", joint.name.c_str(),
                joint.motor_id, joint.position);
  }
  publish_state_values();
  return all_received;
}

RebotArmRsSystem::CallbackReturn RebotArmRsSystem::on_activate(
    const rclcpp_lifecycle::State &) {
  try {
    if (!read_initial_positions() && require_all_motors_) {
      return CallbackReturn::ERROR;
    }
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      if (joints_[index].has_position) {
        set_command(position_command_handles_[index], joints_[index].position, true);
      }
    }

    if (!auto_enable_on_activate_) {
      RCLCPP_WARN(get_logger(), "Activated in read-only mode: motors remain disabled");
      outputs_enabled_ = false;
      return CallbackReturn::SUCCESS;
    }

    for (const auto &joint : joints_) {
      transport_.send(robstride::make_write_parameter_i8(joint.motor_id, host_id_, kRunMode, 0));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(40));
    drain_receive_queue(64);
    for (const auto &joint : joints_) {
      transport_.send(robstride::make_enable(joint.motor_id, host_id_));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    for (const auto &joint : joints_) {
      const double motor_target = joint.direction * (joint.position - joint.offset);
      transport_.send(robstride::make_mit_command(
          joint.motor_id, joint.motor_limits, static_cast<float>(motor_target), 0.0F,
          static_cast<float>(joint.kp), static_cast<float>(joint.kd), 0.0F));
    }
    outputs_enabled_ = true;
    RCLCPP_WARN(get_logger(), "All six RobStride motors enabled in MIT position mode");
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "Activation failed: %s", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

RebotArmRsSystem::CallbackReturn RebotArmRsSystem::on_deactivate(
    const rclcpp_lifecycle::State &) {
  if (outputs_enabled_ && disable_on_deactivate_) {
    try {
      for (const auto &joint : joints_) {
        transport_.send(robstride::make_disable(joint.motor_id, host_id_));
      }
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "Disable failed: %s", error.what());
      return CallbackReturn::ERROR;
    }
  }
  outputs_enabled_ = false;
  return CallbackReturn::SUCCESS;
}

RebotArmRsSystem::CallbackReturn RebotArmRsSystem::on_cleanup(
    const rclcpp_lifecycle::State &) {
  transport_.close();
  return CallbackReturn::SUCCESS;
}

void RebotArmRsSystem::update_joint_position(
    Joint &joint, double motor_position, std::chrono::steady_clock::time_point timestamp) {
  const double next_position = joint.direction * motor_position + joint.offset;
  if (joint.has_position) {
    const double seconds = std::chrono::duration<double>(timestamp - joint.last_position_time).count();
    if (seconds > 1e-6) {
      joint.velocity = (next_position - joint.position) / seconds;
    }
  }
  joint.position = next_position;
  joint.last_position_time = timestamp;
  joint.has_position = true;
}

RebotArmRsSystem::Joint *RebotArmRsSystem::find_joint(std::uint8_t motor_id) {
  const auto iterator = std::find_if(joints_.begin(), joints_.end(),
                                     [motor_id](const Joint &joint) { return joint.motor_id == motor_id; });
  return iterator == joints_.end() ? nullptr : &*iterator;
}

const RebotArmRsSystem::Joint *RebotArmRsSystem::find_joint(std::uint8_t motor_id) const {
  const auto iterator = std::find_if(joints_.begin(), joints_.end(),
                                     [motor_id](const Joint &joint) { return joint.motor_id == motor_id; });
  return iterator == joints_.end() ? nullptr : &*iterator;
}

void RebotArmRsSystem::process_frame(const robstride::Frame &frame) {
  if (const auto parameter = robstride::decode_parameter_response(frame)) {
    if (parameter->parameter_id == kMechanicalPosition) {
      if (auto *joint = find_joint(parameter->device_id)) {
        update_joint_position(*joint, robstride::parameter_as_f32(*parameter),
                              std::chrono::steady_clock::now());
        joint->mechanical_position_received = true;
      }
    }
    return;
  }

  const auto id = robstride::parse_extended_id(frame.arbitration_id);
  if (id.communication_type != robstride::CommunicationType::kOperationStatus) {
    return;
  }
  auto *joint = find_joint(static_cast<std::uint8_t>(id.extra_data & 0xFFU));
  if (joint == nullptr) {
    return;
  }
  if (const auto status = robstride::decode_status(frame, joint->motor_limits)) {
    update_joint_position(*joint, status->position, std::chrono::steady_clock::now());
    joint->effort = joint->direction * status->torque;
    const bool faulted = status->uncalibrated || status->stall || status->magnetic_encoder_fault ||
                         status->overtemperature || status->overcurrent || status->undervoltage;
    if (faulted && !joint->faulted) {
      RCLCPP_ERROR(get_logger(), "Motor 0x%02X reported a fault in status feedback", joint->motor_id);
    }
    joint->faulted = faulted;
  }
}

void RebotArmRsSystem::drain_receive_queue(std::size_t maximum_frames) {
  for (std::size_t count = 0; count < maximum_frames; ++count) {
    const auto frame = transport_.receive(std::chrono::milliseconds(0));
    if (!frame) {
      return;
    }
    process_frame(*frame);
  }
}

void RebotArmRsSystem::publish_state_values() {
  for (std::size_t index = 0; index < joints_.size(); ++index) {
    set_state(position_state_handles_[index], joints_[index].position, false);
    set_state(velocity_state_handles_[index], joints_[index].velocity, false);
    set_state(effort_state_handles_[index], joints_[index].effort, false);
  }
}

void RebotArmRsSystem::send_position_poll() {
  if (joints_.empty()) {
    return;
  }
  const auto &joint = joints_[next_poll_joint_];
  transport_.send(robstride::make_read_parameter(joint.motor_id, host_id_, kMechanicalPosition));
  next_poll_joint_ = (next_poll_joint_ + 1U) % joints_.size();
}

hardware_interface::return_type RebotArmRsSystem::read(const rclcpp::Time &,
                                                       const rclcpp::Duration &) {
  try {
    drain_receive_queue(64);
    if ((read_cycle_++ % position_poll_divider_) == 0U) {
      send_position_poll();
    }
    publish_state_values();
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "CAN read cycle failed: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RebotArmRsSystem::write(const rclcpp::Time &,
                                                        const rclcpp::Duration &) {
  if (!outputs_enabled_) {
    return hardware_interface::return_type::OK;
  }
  try {
    for (std::size_t index = 0; index < joints_.size(); ++index) {
      auto &joint = joints_[index];
      double command = joint.last_sent_position;
      if (!get_command(position_command_handles_[index], command, false) || !std::isfinite(command)) {
        command = joint.last_sent_position;
      }
      command = std::clamp(command, joint.min_position, joint.max_position);
      command = std::clamp(command, joint.last_sent_position - max_command_step_,
                           joint.last_sent_position + max_command_step_);
      const double motor_target = joint.direction * (command - joint.offset);
      transport_.send(robstride::make_mit_command(
          joint.motor_id, joint.motor_limits, static_cast<float>(motor_target), 0.0F,
          static_cast<float>(joint.kp), static_cast<float>(joint.kd), 0.0F));
      joint.last_sent_position = command;
    }
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "CAN write cycle failed: %s", error.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace rebot_arm_rs_hardware

PLUGINLIB_EXPORT_CLASS(rebot_arm_rs_hardware::RebotArmRsSystem,
                       hardware_interface::SystemInterface)
