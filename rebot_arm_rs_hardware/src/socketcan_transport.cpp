#include "rebot_arm_rs_hardware/socketcan_transport.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <iterator>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace rebot_arm_rs_hardware {
namespace {

std::runtime_error socket_error(const std::string &operation, const std::string &interface_name) {
  return std::runtime_error(operation + " on " + interface_name + ": " + std::strerror(errno));
}

}  // namespace

SocketCanTransport::~SocketCanTransport() { close(); }

void SocketCanTransport::open(const std::string &interface_name) {
  close();
  interface_name_ = interface_name;
  socket_fd_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socket_fd_ < 0) {
    throw socket_error("socket(PF_CAN) failed", interface_name_);
  }

  struct ifreq request {};
  std::strncpy(request.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
  if (::ioctl(socket_fd_, SIOCGIFINDEX, &request) < 0) {
    const auto error = socket_error("SIOCGIFINDEX failed", interface_name_);
    close();
    throw error;
  }

  struct sockaddr_can address {};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (::bind(socket_fd_, reinterpret_cast<struct sockaddr *>(&address), sizeof(address)) < 0) {
    const auto error = socket_error("bind failed", interface_name_);
    close();
    throw error;
  }

  const int flags = ::fcntl(socket_fd_, F_GETFL, 0);
  if (flags < 0 || ::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
    const auto error = socket_error("setting nonblocking mode failed", interface_name_);
    close();
    throw error;
  }
}

void SocketCanTransport::close() {
  if (socket_fd_ >= 0) {
    ::close(socket_fd_);
    socket_fd_ = -1;
  }
}

void SocketCanTransport::send(const robstride::Frame &frame) const {
  if (socket_fd_ < 0) {
    throw std::runtime_error("SocketCAN transport is not open");
  }
  if (!frame.extended || frame.arbitration_id > CAN_EFF_MASK || frame.dlc > CAN_MAX_DLEN) {
    throw std::invalid_argument("invalid RobStride extended CAN frame");
  }

  struct can_frame raw {};
  raw.can_id = frame.arbitration_id | CAN_EFF_FLAG;
  raw.len = frame.dlc;
  std::copy(frame.data.begin(), frame.data.end(), raw.data);
  if (::write(socket_fd_, &raw, sizeof(raw)) != static_cast<ssize_t>(sizeof(raw))) {
    throw socket_error("CAN write failed", interface_name_);
  }
}

std::optional<robstride::Frame> SocketCanTransport::receive(std::chrono::milliseconds timeout) const {
  if (socket_fd_ < 0) {
    throw std::runtime_error("SocketCAN transport is not open");
  }

  struct pollfd descriptor {socket_fd_, POLLIN, 0};
  const int result = ::poll(&descriptor, 1, static_cast<int>(timeout.count()));
  if (result < 0) {
    if (errno == EINTR) {
      return std::nullopt;
    }
    throw socket_error("CAN poll failed", interface_name_);
  }
  if (result == 0) {
    return std::nullopt;
  }

  struct can_frame raw {};
  const auto bytes = ::read(socket_fd_, &raw, sizeof(raw));
  if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
    return std::nullopt;
  }
  if (bytes != static_cast<ssize_t>(sizeof(raw))) {
    throw socket_error("CAN read failed", interface_name_);
  }

  robstride::Frame frame;
  frame.extended = (raw.can_id & CAN_EFF_FLAG) != 0U;
  frame.arbitration_id = raw.can_id & (frame.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
  frame.dlc = std::min<std::uint8_t>(raw.len, CAN_MAX_DLEN);
  std::copy(std::begin(raw.data), std::end(raw.data), frame.data.begin());
  return frame;
}

}  // namespace rebot_arm_rs_hardware
