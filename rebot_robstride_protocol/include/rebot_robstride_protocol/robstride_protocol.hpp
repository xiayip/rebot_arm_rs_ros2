#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "rebot_robstride_protocol/robstride_codec.hpp"
#include "shared_can_interface/enable_readiness_interface.hpp"
#include "shared_can_interface/protocol_interface.hpp"

namespace rebot_robstride_protocol {

class RobStrideProtocol final
    : public shared_can_interface::v1::ProtocolInterface,
      public shared_can_interface::v1::EnableReadinessInterface {
public:
  void
  configure(const shared_can_interface::v1::ParameterMap &bus_parameters,
            const std::vector<shared_can_interface::v1::DeviceConfiguration>
                &devices) override;

  void reset_state() override;
  std::vector<shared_can_interface::v1::CanFrame>
  initial_state_requests() const override;
  bool initial_state_ready() const override;
  std::vector<std::string> missing_initial_states() const override;
  bool process_frame(const shared_can_interface::v1::CanFrame &frame,
                     std::chrono::steady_clock::time_point timestamp) override;
  std::optional<shared_can_interface::v1::CanFrame> periodic_request() override;
  std::vector<shared_can_interface::v1::TimedFrameBatch>
  pre_enable_sequence() const override;
  void begin_enable_check(std::chrono::steady_clock::time_point now) override;
  std::vector<std::string> pending_enable_states(
      std::chrono::steady_clock::time_point now) const override;
  std::vector<shared_can_interface::v1::TimedFrameBatch>
  enable_sequence() const override;
  std::vector<shared_can_interface::v1::CanFrame>
  disable_frames() const override;
  bool set_command(const std::string &device_name,
                   const std::string &interface_name, double value) override;
  std::size_t command_frame_capacity() const noexcept override;
  void command_frames(
      std::vector<shared_can_interface::v1::CanFrame> &output) override;
  bool get_state(const std::string &device_name,
                 const std::string &interface_name,
                 double &value) const override;
  bool get_initial_command(const std::string &device_name,
                           const std::string &interface_name,
                           double &value) const override;
  bool faulted(const std::string &device_name) const override;

private:
  struct Device {
    std::string name;
    std::uint8_t motor_id{0};
    codec::MotorLimits limits;
    double direction{1.0};
    double offset{0.0};
    double mit_kp{0.0};
    double mit_kd{0.0};
    double position{0.0};
    double velocity{0.0};
    double effort{0.0};
    double temperature_c{0.0};
    double command_position{std::numeric_limits<double>::quiet_NaN()};
    double command_velocity{0.0};
    double command_t_ff{0.0};
    double command_kp{0.0};
    double command_kd{0.0};
    double torque_constant_nm_per_arms{0.0};
    double filtered_current_peak{0.0};
    double startup_preload_max{0.05};
    std::uint8_t run_mode{0};
    std::uint8_t mode_state{0};
    bool position_valid{false};
    bool status_received{false};
    bool mechanical_position_received{false};
    bool filtered_current_received{false};
    bool run_mode_received{false};
    bool faulted{false};
    std::chrono::steady_clock::time_point last_status{};
    std::chrono::steady_clock::time_point stable_since{};
    std::size_t stable_samples{0};
    double stable_position_min{0.0};
    double stable_position_max{0.0};
    double stable_temperature_min{0.0};
    double stable_temperature_max{0.0};
    double check_initial_position{0.0};
    bool startup_moved{false};
  };

  Device *find_by_motor_id(std::uint8_t motor_id);
  Device *find_by_name(const std::string &name);
  const Device *find_by_name(const std::string &name) const;
  std::uint8_t host_id_{0xFD};
  std::vector<Device> devices_;
  bool enable_check_started_{false};
  std::chrono::steady_clock::time_point enable_check_start_{};
};

} // namespace rebot_robstride_protocol
