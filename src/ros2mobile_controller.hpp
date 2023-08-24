#pragma once

#include <arpa/inet.h>
#include <bits/stdc++.h>
#include <netinet/in.h>
#include <rclcpp/rclcpp.hpp>
#include <ros2usb_msgs/msg/usb_packet.hpp>
#include <sys/socket.h>
#include <sys/types.h>

class Ros2MobileController : public rclcpp::Node {
public:
  Ros2MobileController();
  ~Ros2MobileController();
  bool connectToController();

private:
  void topic_callback(const ros2usb_msgs::msg::USBPacket &msg);
  void sendToController(const ros2usb_msgs::msg::USBPacket::SharedPtr &msg);
  bool connected = false;
  int sender_socket_fd;
  int receiver_socket_fd;
  struct sockaddr_in sender_addr;
  struct sockaddr_in receiver_addr;
  std::string local_address = "0.0.0.0";
  std::string remote_address = "";
  int local_port = 10000;
  int remote_port = 10001;
  rclcpp::Subscription<ros2usb_msgs::msg::USBPacket>::SharedPtr subscription_;
  rclcpp::Publisher<ros2usb_msgs::msg::USBPacket>::SharedPtr publisher_;
  const std::string sub_topic = "ros2micon";
};
