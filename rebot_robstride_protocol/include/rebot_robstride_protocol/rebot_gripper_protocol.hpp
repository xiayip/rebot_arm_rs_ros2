#pragma once

#include <chrono>
#include <limits>
#include <string>
#include <vector>

#include "rebot_robstride_protocol/robstride_protocol.hpp"
#include "shared_can_interface/enable_readiness_interface.hpp"
#include "shared_can_interface/protocol_interface.hpp"

namespace rebot_robstride_protocol
{

// Adapts the official RS00 gripper motor to a normalized logical joint:
// 0.0 is fully open and 1.0 is fully closed. The underlying RobStride
// protocol remains responsible for MIT encoding, safe startup and feedback.
class RebotGripperProtocol final
  : public shared_can_interface::v1::ProtocolInterface,
    public shared_can_interface::v1::EnableReadinessInterface
{
public:
  void configure(
    const shared_can_interface::v1::ParameterMap & bus_parameters,
    const std::vector<shared_can_interface::v1::DeviceConfiguration> & devices) override;

  void reset_state() override;
  std::vector<shared_can_interface::v1::CanFrame> initial_state_requests() const override;
  bool initial_state_ready() const override;
  std::vector<std::string> missing_initial_states() const override;
  bool process_frame(
    const shared_can_interface::v1::CanFrame & frame,
    std::chrono::steady_clock::time_point timestamp) override;
  std::optional<shared_can_interface::v1::CanFrame> periodic_request() override;
  std::vector<shared_can_interface::v1::TimedFrameBatch> pre_enable_sequence() const override;
  void begin_enable_check(std::chrono::steady_clock::time_point now) override;
  std::vector<std::string> pending_enable_states(
    std::chrono::steady_clock::time_point now) const override;
  std::vector<shared_can_interface::v1::TimedFrameBatch> enable_sequence() const override;
  std::vector<shared_can_interface::v1::CanFrame> disable_frames() const override;
  bool set_command(
    const std::string & device_name, const std::string & interface_name,
    double value) override;
  std::size_t command_frame_capacity() const noexcept override;
  void command_frames(std::vector<shared_can_interface::v1::CanFrame> & output) override;
  bool get_state(
    const std::string & device_name, const std::string & interface_name,
    double & value) const override;
  bool get_initial_command(
    const std::string & device_name, const std::string & interface_name,
    double & value) const override;
  bool faulted(const std::string & device_name) const override;

private:
  struct Device
  {
    std::string name;
    double open_position{0.0};
    double closed_position{0.0};
    double default_velocity{0.25};
    double target_position{std::numeric_limits<double>::quiet_NaN()};
    double sent_position{std::numeric_limits<double>::quiet_NaN()};
    double velocity_limit{0.25};
    std::chrono::steady_clock::time_point last_command_time{};
  };

  Device * find(const std::string & name);
  const Device * find(const std::string & name) const;
  double to_normalized(const Device & device, double motor_position) const;
  double to_motor(const Device & device, double normalized_position) const;

  RobStrideProtocol motor_protocol_;
  std::vector<Device> devices_;
};

}  // namespace rebot_robstride_protocol
