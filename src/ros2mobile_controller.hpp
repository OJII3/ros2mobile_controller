#pragma once

#include <algorithm>
#include <arpa/inet.h>
#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <ros2usb_msgs/msg/usb_packet.hpp>
#include <vector>

#include "udp_client/udp_client.hpp"

class ROS2MobileController : public rclcpp::Node {
public:
  enum ConnectionState {
    Connected,
    Disconnected,
    Connecting,
  };
  ConnectionState connectionState = Disconnected;
  ROS2MobileController();
  ~ROS2MobileController();
  bool connect();
  void update(); // timtask

private:
};
