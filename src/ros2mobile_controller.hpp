#pragma once

#include <arpa/inet.h>
#include <bits/stdc++.h>
#include <netinet/in.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2usb_msgs/msg/usb_packet.hpp>
#include <sys/socket.h>
#include <sys/types.h>

// TODO: create UDPClient class and use it in this class
class ROS2MobileController : public rclcpp::Node {
public:
  enum ConnectionState {
    Connected,
    Disconnected,
    Connecting,
  };
  ConnectionState connection_state = Disconnected;
  ROS2MobileController();
  ~ROS2MobileController();
  void connect();
  void send(const std::vector<std::byte> &buffer);
  void Receive(std::array<std::byte, 1024> &buffer);

private:
  void topicCallback(const ros2usb_msgs::msg::USBPacket &msg);
  void rosmsgToBytes(const ros2usb_msgs::msg::USBPacket &msg,
                     std::vector<std::byte> &result_buffer);
  int socket_fd;
  struct sockaddr_in local_addr;
  struct sockaddr_in remote_addr;
  socklen_t local_addr_len = sizeof(local_addr);
  socklen_t remote_addr_len = sizeof(remote_addr);
  const std::string local_address = "0.0.0.0";
  std::string remote_address = "";
  int local_port = 10000;
  int remote_port = 10001;
  rclcpp::Subscription<ros2usb_msgs::msg::USBPacket>::SharedPtr subscription_;
  rclcpp::Publisher<ros2usb_msgs::msg::USBPacket>::SharedPtr publisher_;
  const char *ping_message = "ping-robot";
  const char *pong_message = "pong-robot";
  const std::string sub_topic = "ros2micon";
};
