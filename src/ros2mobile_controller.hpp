#pragma once

#include <algorithm>
#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <ros2usb_msgs/msg/usb_packet.hpp>
#include <vector>

#include "udp_client/udp_client.hpp"

class ROS2MobileController : public rclcpp::Node {
 public:
  ROS2MobileController();

  enum ConnectionState {
    Connected,
    Disconnected,
    Connecting,
  };

  ConnectionState getConnectionState() { return connection_state_; }
  void update();  // timtask

 private:
  static constexpr size_t rosmsg_size = 1024;
  static constexpr uint16_t local_port = 10000;
  static constexpr uint16_t remote_port = 10001;
  ConnectionState connection_state_ = Disconnected;
  std::string ping_message_ = "ping-rur";
  UDPClient udp_client_;
  std::string local_ip_ = "0.0.0.0";
  std::string remote_ip_;
  rclcpp::Publisher<ros2usb_msgs::msg::USBPacket>::SharedPtr publisher_;

  void connect();
};
