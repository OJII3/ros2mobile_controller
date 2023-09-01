#include "ros2mobile_controller.hpp"

ROS2MobileController::ROS2MobileController()
    : rclcpp::Node("ros2mobile_controller"),
      remote_endpoint_(asio::ip::udp::v4(), remote_port),
      receive_socket_(io_service_, asio::ip::udp::endpoint(asio::ip::udp::v4(),
                                                           local_receive_port)),
      send_socket_(io_service_),
      publisher_(this->create_publisher<ros2usb_msgs::msg::USBPacket>(
          "ros2micon", qos)),
      subscription_(this->create_subscription<ros2usb_msgs::msg::USBPacket>(
          subscription_topic_, qos,
          std::bind(&ROS2MobileController::subscriptionCallback, this,
                    std::placeholders::_1))) {
  send_socket_.open(asio::ip::udp::v4());
};

void ROS2MobileController::update() {
  // concat buffer to string by comma
  auto now = std::chrono::steady_clock::now();
  if (now - last_received_time_ >
      std::chrono::milliseconds(connection_timeout_ms)) {
    connection_state_ = Disconnected;
  }

  switch (connection_state_) {
    case Connected: {
      if (send_buffer_.size() == 0) {
        send_buffer_.resize(ping_message_.size());
        memcpy(send_buffer_.data(), ping_message_.c_str(),
               ping_message_.size());
      }
      send_socket_.send_to(asio::buffer(send_buffer_), remote_endpoint_);
      send_buffer_.clear();
      receive_buffer_.resize(data_size_default);
      /* receive_socket_.async_receive_from( */
      /*     asio::buffer(receive_buffer_), remote_endpoint_, */
      /*     std::bind(&ROS2MobileController::onReceive, this, */
      /*               std::placeholders::_1, std::placeholders::_2)); */
      auto len = receive_socket_.receive_from(asio::buffer(receive_buffer_),
                                              remote_endpoint_);
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "received: " << len << " "
                                      << std::string(receive_buffer_.begin(),
                                                     receive_buffer_.end()));
      receive_buffer_.resize(len);
      if (std::string(receive_buffer_.begin(), receive_buffer_.end()) ==
          ping_message_) {
        last_received_time_ = std::chrono::steady_clock::now();
      } else {
        auto msg = ros2usb_msgs::msg::USBPacket();
        msg.id.data = 1;
        msg.packet.data.resize(receive_buffer_.size());
        memcpy(msg.packet.data.data(), receive_buffer_.data(),
               receive_buffer_.size());
        publisher_->publish(msg);
      }
      break;
    }
    case Disconnected: {
      this->connect();
      break;
    }
    case Connecting: {
      // do nothing and wait for connection
      break;
    }
  }
}

void ROS2MobileController::connect() {
  connection_state_ = Connecting;
  auto temp_endpoint =
      asio::ip::udp::endpoint(asio::ip::udp::v4(), remote_port);
  receive_buffer_.resize(data_size_default);

  while (rclcpp::ok() && connection_state_ == Connecting) {
    if (remote_endpoint_.size() != 0) {
      send_socket_.send_to(asio::buffer(ping_message_), remote_endpoint_);
    }
    auto len = receive_socket_.receive_from(asio::buffer(receive_buffer_),
                                            temp_endpoint);
    receive_buffer_.resize(len);
    if (std::string(receive_buffer_.begin(), receive_buffer_.end()) ==
        ping_message_) {
      last_received_time_ = std::chrono::steady_clock::now();
      remote_endpoint_ = temp_endpoint;
      connection_state_ = Connected;
      break;
    }
  }
};

void ROS2MobileController::subscriptionCallback(
    const ros2usb_msgs::msg::USBPacket &msg) {
  send_buffer_.resize(sizeof(msg.packet) + 1);
  send_buffer_[0] = msg.id.data;
  std::copy(msg.packet.data.begin(), msg.packet.data.end(),
            send_buffer_.begin() + 1);
}

void ROS2MobileController::onReceive(asio::error_code error,
                                     size_t bytes_transferred) {
  if (error) {
    RCLCPP_ERROR_STREAM(this->get_logger(), error.message());
    return;
  }
  receive_buffer_.resize(bytes_transferred);
  if (std::string(receive_buffer_.begin(), receive_buffer_.end()) ==
      ping_message_) {
    last_received_time_ = std::chrono::steady_clock::now();
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "received: " << std::string(receive_buffer_.begin(),
                                                   receive_buffer_.end()));
  }
}
