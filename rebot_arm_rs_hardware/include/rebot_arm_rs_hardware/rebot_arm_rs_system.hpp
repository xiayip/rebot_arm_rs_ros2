#pragma once

#include <chrono>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rebot_arm_rs_hardware/robstride_protocol.hpp"
#include "rebot_arm_rs_hardware/socketcan_transport.hpp"

namespace rebot_arm_rs_hardware {

class RebotArmRsSystem final : public hardware_interface::SystemInterface {
 public:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams &params) override;
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface::SharedPtr> on_export_command_interfaces() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;
  hardware_interface::return_type write(const rclcpp::Time &time,
                                        const rclcpp::Duration &period) override;

 private:
  struct Joint {
    std::string name;
    std::uint8_t motor_id{0};
    std::string model;
    robstride::MotorLimits motor_limits;
    double direction{1.0};
    double offset{0.0};
    double kp{0.0};
    double kd{0.0};
    double min_position{0.0};
    double max_position{0.0};
    double position{std::numeric_limits<double>::quiet_NaN()};
    double velocity{0.0};
    double effort{0.0};
    double last_sent_position{std::numeric_limits<double>::quiet_NaN()};
    std::chrono::steady_clock::time_point last_position_time{};
    bool has_position{false};
    bool mechanical_position_received{false};
    bool faulted{false};
  };

  bool read_initial_positions();
  void process_frame(const robstride::Frame &frame);
  void update_joint_position(Joint &joint, double motor_position,
                             std::chrono::steady_clock::time_point timestamp);
  Joint *find_joint(std::uint8_t motor_id);
  const Joint *find_joint(std::uint8_t motor_id) const;
  void publish_state_values();
  void drain_receive_queue(std::size_t maximum_frames);
  void send_position_poll();

  std::string can_interface_{"can1"};
  std::uint8_t host_id_{0xFD};
  bool auto_enable_on_activate_{false};
  bool require_all_motors_{true};
  bool disable_on_deactivate_{true};
  bool outputs_enabled_{false};
  int initial_read_timeout_ms_{1200};
  std::size_t position_poll_divider_{1};
  std::size_t read_cycle_{0};
  std::size_t next_poll_joint_{0};
  double max_command_step_{0.02};

  SocketCanTransport transport_;
  std::vector<Joint> joints_;
  std::vector<hardware_interface::StateInterface::SharedPtr> position_state_handles_;
  std::vector<hardware_interface::StateInterface::SharedPtr> velocity_state_handles_;
  std::vector<hardware_interface::StateInterface::SharedPtr> effort_state_handles_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> position_command_handles_;
};

}  // namespace rebot_arm_rs_hardware
