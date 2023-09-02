#include "ros2mobile_controller.hpp"

#include <ros2usb_msgs/msg/detail/usb_packet__struct.hpp>

ROS2MobileController::ROS2MobileController()
    : Node("ros2mobile_controller"),
      watchdog_(std::make_shared<Watchdog>()),
      udp_listener_(std::make_shared<UDPListener>(local_listen_port_default)),
      udp_broadcaster_(
          std::make_shared<UDPBroadcaster>(local_send_port_default)),
      publisher_(
          create_publisher<ros2usb_msgs::msg::USBPacket>(topic_name_, qos)),
      subscription_(create_subscription<ros2usb_msgs::msg::USBPacket>(
          topic_name_, qos,
          std::bind(&ROS2MobileController::subscriptionCallback, this,
                    std::placeholders::_1))) {}

void ROS2MobileController::start() {
  try {
    // start watching for connection
    auto watchdog_async = std::async(std::launch::async, [&]() -> void {
      watchdog_->startLoop(
          std::chrono::milliseconds(watch_interval_ms_default),
          std::chrono::milliseconds(watch_timeout_ms_default), [&]() -> void {
            if (connection_ == Connection::CONNECTED) {
              connection_ = Connection::DISCONNECTED;
              RCLCPP_INFO_STREAM(get_logger(), "watchdog: connection lost");
            }
          });
    });

    // start listening for messages
    auto listener_async = std::async(std::launch::async, [&]() -> void {
      udp_listener_->startReceiveLoop(
          max_buffer_size_default,
          [&](const UDPListener::ReceiveResult &result) -> void {
            if (result.buffer_.size() > 4 && result.buffer_[1] == 'S' &&
                result.buffer_[2] == 'S' &&
                result.buffer_[result.buffer_.size() - 1] == 'E' &&
                result.buffer_[result.buffer_.size() - 2] == 'E') {
              connection_ = Connection::CONNECTED;
              watchdog_->update();
              auto msg = ros2usb_msgs::msg::USBPacket();
              msg.id.data = result.buffer_[0];
              msg.packet.data.resize(result.buffer_.size() - 1);
              copy(result.buffer_.begin() + 1, result.buffer_.end(),
                   msg.packet.data.begin());
              publisher_->publish(msg);
              RCLCPP_INFO_STREAM(get_logger(), "listen: received "
                                                   << result.buffer_.size()
                                                   << " bytes from "
                                                   << result.remote_endpoint_);
            }
          });
    });

    // start broadcasting messages
    auto broadcast_async = std::async(std::launch::async, [&]() -> void {
      udp_broadcaster_->startBroadcastLoop(
          remote_listen_port_default,
          std::chrono::milliseconds(broadcast_interval_ms_default));
    });

    udp_broadcaster_->updateBuffer({'0', 'S', 'S', 'p', 'i', 'n', 'g', '-', 'r',
                                    'o', 'b', 'o', 't', 'E', 'E'});
  } catch (...) {
    this->shutdown();
    RCLCPP_ERROR(get_logger(), "Exception in ros2mobile_controller");
  }
}

void ROS2MobileController::subscriptionCallback(
    const ros2usb_msgs::msg::USBPacket &msg) {
  if (connection_ == Connection::CONNECTED) {
    auto buffer =
        std::vector<uint8_t>(msg.packet.data.begin(), msg.packet.data.end());
    buffer.insert(buffer.begin(), msg.id.data);
    udp_broadcaster_->updateBuffer(buffer);
  }
};

void ROS2MobileController::stop() {
  watchdog_->stopLoop();
  udp_listener_->stopLoop();
  udp_broadcaster_->stopLoop();
}

void ROS2MobileController::shutdown() {
  watchdog_ = nullptr;
  udp_listener_ = nullptr;
  udp_broadcaster_ = nullptr;
}
