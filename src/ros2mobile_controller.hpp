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

// TODO: create UDPClient class and use it in this class
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
  void start();  // start timtask
  void update(); // timtask

private:
  void send(const std::vector<int8_t> &buffer);
  std::vector<int8_t> receive();
  void topicCallback(const ros2usb_msgs::msg::USBPacket &msg);
  std::vector<int8_t> rosmsgToBytes(const ros2usb_msgs::msg::USBPacket &msg);
  void initSocket();
  int socket_fd;
  struct sockaddr_in local_addr;
  struct sockaddr_in remote_addr;
  socklen_t local_addr_len = sizeof(local_addr);
  socklen_t remote_addr_len = sizeof(remote_addr);
  const std::string local_address = "0.0.0.0";
  std::string remote_address = "";
  int local_port = 10000;
  int remote_port = 10001;
  int connectionFailCount = 0;
  std::vector<int8_t> send_buffer;
  std::vector<int8_t> receive_buffer;
  const int maxConnectionFailCount = 10;
  rclcpp::Subscription<ros2usb_msgs::msg::USBPacket>::SharedPtr subscription_;
  rclcpp::Publisher<ros2usb_msgs::msg::USBPacket>::SharedPtr publisher_;
  const std::string ping_message = "ping-robot";
  const std::string sub_topic = "ros2micon";
};
