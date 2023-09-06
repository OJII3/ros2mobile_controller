#pragma once

#include <chrono>
#include <rclcpp/rclcpp.hpp>

#include "ros2usb_msgs/msg/usb_packet.hpp"
#include "udp_broadcaster/udp_broadcaster.hpp"
#include "udp_listener/udp_listener.hpp"
#include "watchdog/watchdog.hpp"

class ROS2MobileController : public rclcpp::Node {
 public:
  enum class Connection { CONNECTED, DISCONNECTED };

  ROS2MobileController();
  void start();
  void stop();

 private:
  static constexpr int qos = 1024;
  static constexpr int max_buffer_size_default = 1024;
  static constexpr uint16_t local_listen_port_default = 10000;
  static constexpr uint16_t local_send_port_default = 5000;  // not important
  static constexpr uint16_t remote_listen_port_default = 10001;
  static constexpr int watch_interval_ms_default = 5;
  static constexpr int watch_timeout_ms_default = 2000;
  static constexpr int broadcast_interval_ms_default = 5;

  std::string sub_topic_name_ = "micon2ros";
  std::string pub_topic_name_ = "ros2micon";

  Connection connection_ = Connection::DISCONNECTED;
  std::shared_ptr<Watchdog> watchdog_;
  std::shared_ptr<UDPListener> udp_listener_;
  std::shared_ptr<UDPBroadcaster> udp_broadcaster_;
  rclcpp::Publisher<ros2usb_msgs::msg::USBPacket>::SharedPtr publisher_;
  rclcpp::Subscription<ros2usb_msgs::msg::USBPacket>::SharedPtr subscription_;

  void shutdown();
  void subscriptionCallback(const ros2usb_msgs::msg::USBPacket &msg);
};
