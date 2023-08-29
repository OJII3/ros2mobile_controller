#include "ros2mobile_controller.hpp"
#include <cstdint>

using std::placeholders::_1;

ROS2MobileController::ROS2MobileController() : Node("ros2mobile_controller") {
  subscription_ = this->create_subscription<ros2usb_msgs::msg::USBPacket>(
      sub_topic, 10, bind(&ROS2MobileController::topicCallback, this, _1));
  this->initSocket();
}

ROS2MobileController::~ROS2MobileController() { close(socket_fd); }

void ROS2MobileController::start() {}

void ROS2MobileController::update() {
  switch (connectionState) {
  case ConnectionState::Disconnected: {
    RCLCPP_INFO_STREAM(this->get_logger(), "Disconnected");
    this->connect();
    return;
  }
  case ConnectionState::Connecting: {
    return;
  }
  case ConnectionState::Connected: {
    // Send message set in previous loop
    send(send_buffer);
    RCLCPP_INFO_STREAM(this->get_logger(), "Connected");
    if (connectionFailCount > maxConnectionFailCount) {
      connectionState = ConnectionState::Disconnected;
      connectionFailCount = 0;
    }
    auto buffer = this->receive();
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Received message" << buffer[0] << buffer[1]);

    if (buffer.size() <= 0) {
      connectionFailCount++;
      RCLCPP_INFO_STREAM(this->get_logger(), "Connection Failed");
    } else if (memcmp(buffer.data(), ping_message.data(),
                      ping_message.size()) == 0) {
      send_buffer.resize(ping_message.size());
      memcpy(&send_buffer, ping_message.data(), ping_message.size());
    } else {
      send_buffer.resize(buffer.size());
      memcpy(&send_buffer, buffer.data(), buffer.size());
      ros2usb_msgs::msg::USBPacket msg;
      msg.id.data = 1;
      msg.packet.data.clear();
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Sending message to topic " << sub_topic);
      msg.packet.data.resize(buffer.size());
      memcpy(msg.packet.data.data(), buffer.data(), buffer.size());
      RCLCPP_INFO_STREAM(this->get_logger(), msg.packet.data.size());
      /* publisher_->publish(msg); */
      /* RCLCPP_INFO_STREAM(this->get_logger(), "Message sent"); */
    }
  }
    return;
  }
}

bool ROS2MobileController::connect() {
  connectionState = ConnectionState::Connecting;

  while (rclcpp::ok() && connectionState == ConnectionState::Connecting) {

    int8_t buffer[1024];
    memset(buffer, 0, sizeof(buffer));
    sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);
    auto size = recvfrom(socket_fd, buffer, sizeof(buffer), 0,
                         (sockaddr *)&addr, &addr_len);

    if (size < 0 || memcmp(buffer, ping_message.data(), size) != 0) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Received Invalid Request.");
      continue;
    } else {
      // Get the remote address and port
      remote_addr = addr;
      remote_address = inet_ntoa(addr.sin_addr);
      remote_port = ntohs(addr.sin_port);
      sendto(socket_fd, ping_message.data(), ping_message.size(), 0,
             (sockaddr *)&remote_addr, sizeof(remote_addr));
      connectionState = ConnectionState::Connected;
      RCLCPP_INFO_STREAM(this->get_logger(), "Connected to Controller");
      break;
    }
  }
  return true;
}

void ROS2MobileController::send(const std::vector<int8_t> &buffer) {
  if (connectionState == ConnectionState::Connected) {
    sendto(socket_fd, buffer.data(), buffer.size(), 0, (sockaddr *)&local_addr,
           sizeof(local_addr));
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "Not connected");
  }
}

std::vector<int8_t> ROS2MobileController::receive() {
  int8_t buffer[1024];
  if (connectionState == ConnectionState::Connected) {
    recvfrom(socket_fd, &buffer, sizeof(buffer), 0, (sockaddr *)&remote_addr,
             &remote_addr_len);
    receive_buffer.clear();
    receive_buffer.resize(sizeof(buffer));
    memcpy(receive_buffer.data(), &buffer, sizeof(buffer));
    return receive_buffer;
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "Not connected");
  }
  return std::vector<int8_t>{};
}

std::vector<int8_t>
ROS2MobileController::rosmsgToBytes(const ros2usb_msgs::msg::USBPacket &msg) {
  std::vector<int8_t> result_buffer;
  auto id = static_cast<int8_t>(msg.id.data);
  result_buffer.resize(msg.packet.data.size() + 1);
  memcpy(result_buffer.data(), &id, 1);
  memcpy(result_buffer.data() + 1, msg.packet.data.data(),
         msg.packet.data.size());
  return result_buffer;
}

void ROS2MobileController::topicCallback(
    const ros2usb_msgs::msg::USBPacket &msg) {
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Received message from topic " << sub_topic);
  auto buffer = rosmsgToBytes(msg);
  send_buffer.resize(buffer.size());
  memcpy(&send_buffer, buffer.data(), buffer.size());
}

/**
 * @brief Call this in the constructor
 */
void ROS2MobileController::initSocket() {
  close(socket_fd);
  socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = inet_addr(local_address.c_str());
  local_addr.sin_port = htons(local_port);

  // Bind the socket to the local address and port
  auto bind_result =
      bind(socket_fd, (sockaddr *)&local_addr, sizeof(local_addr));

  if (bind_result < 0) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to bind port.");
  }

  RCLCPP_INFO_STREAM(this->get_logger(), "Socket initialized");
}
