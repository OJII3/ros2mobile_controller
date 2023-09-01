#pragma once

#include <asio.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2usb_msgs/msg/usb_packet.hpp>

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
  void onReceive(asio::error_code error, size_t bytes_transferred);
  static constexpr size_t data_size_default = 1024;
  static constexpr size_t qos = 1024;
  static constexpr uint16_t local_receive_port = 10000;
  static constexpr uint16_t remote_port = 10001;
  static const int connection_timeout_ms = 3000;
  ConnectionState connection_state_ = Disconnected;
  asio::io_service io_service_;
  asio::ip::udp::endpoint remote_endpoint_;
  asio::ip::udp::socket receive_socket_;
  asio::ip::udp::socket send_socket_;
  std::string ping_message_ = "ping-robot";
  std::vector<uint8_t> send_buffer_;
  std::vector<uint8_t> receive_buffer_;
  std::chrono::steady_clock::time_point last_received_time_;
  std::string subscription_topic_ = "micon2ros";
  rclcpp::Publisher<ros2usb_msgs::msg::USBPacket>::SharedPtr publisher_;
  rclcpp::Subscription<ros2usb_msgs::msg::USBPacket>::SharedPtr subscription_;
  void subscriptionCallback(const ros2usb_msgs::msg::USBPacket &msg);

  void connect();
};
