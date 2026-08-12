#pragma once

#include <chrono>
#include <optional>
#include <string>

#include "rebot_arm_rs_hardware/robstride_protocol.hpp"

namespace rebot_arm_rs_hardware {

class SocketCanTransport {
 public:
  SocketCanTransport() = default;
  ~SocketCanTransport();

  SocketCanTransport(const SocketCanTransport &) = delete;
  SocketCanTransport &operator=(const SocketCanTransport &) = delete;

  void open(const std::string &interface_name);
  void close();
  [[nodiscard]] bool is_open() const noexcept { return socket_fd_ >= 0; }

  void send(const robstride::Frame &frame) const;
  std::optional<robstride::Frame> receive(std::chrono::milliseconds timeout) const;

 private:
  int socket_fd_{-1};
  std::string interface_name_;
};

}  // namespace rebot_arm_rs_hardware

