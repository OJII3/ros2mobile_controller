#pragma once

#include <algorithm>
#include <arpa/inet.h>
#include <bits/stdc++.h>
#include <cstddef>
#include <cstdint>
#include <netinet/in.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2usb_msgs/msg/usb_packet.hpp>
#include <sys/socket.h>
#include <sys/types.h>
#include <vector>

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
