#include "ros2mobile_controller.hpp"

ROS2MobileController::ROS2MobileController()
    : rclcpp::Node("ros2mobile_controller"),
      udp_client_(local_ip_, local_port),
      publisher_(this->create_publisher<ros2usb_msgs::msg::USBPacket>(
          "usb_packet", rosmsg_size)) {}

void ROS2MobileController::update() {
  switch (connection_state_) {
    case Connected: {
      auto udp_receive_result = udp_client_.receive();
      break;
    }
    case Disconnected:
      this->connect();
      break;
    case Connecting:
      break;
  }
}

void ROS2MobileController::connect() {
  connection_state_ = Connecting;

  while (rclcpp::ok() && connection_state_ == Connecting) {
    if (remote_ip_ != "") {
      udp_client_.sendto(ping_message_, remote_ip_, remote_port);
    }
    auto udp_receive_result = udp_client_.receive();
    if (udp_receive_result.data_str_ == ping_message_) {
      remote_ip_ = udp_receive_result.remote_ip_;
      connection_state_ = Connected;
      break;
    }
  }
};
