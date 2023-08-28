#include "ros2mobile_controller.hpp"
#include <sys/socket.h>

using std::placeholders::_1;

ROS2MobileController::ROS2MobileController() : Node("ros2mobile_controller") {
  subscription_ = this->create_subscription<ros2usb_msgs::msg::USBPacket>(
      sub_topic, 10, bind(&ROS2MobileController::topicCallback, this, _1));
}

ROS2MobileController::~ROS2MobileController() { close(socket_fd); }

void ROS2MobileController::connect() {
  connection_state = ConnectionState::Connecting;

  socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = inet_addr(local_address.c_str());
  local_addr.sin_port = htons(local_port);

  while (rclcpp::ok()) {
    // Bind the socket to the local address and port
    auto bind_result =
        bind(socket_fd, (sockaddr *)&local_addr, sizeof(local_addr));

    if (bind_result < 0) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Failed to bind port. Retrying...");
      rclcpp::sleep_for(std::chrono::seconds(5));
      return;
    }
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "Listening on " << inet_ntoa(local_addr.sin_addr) << ":"
                                       << ntohs(local_addr.sin_port));

    char buffer[1024];
    memset(buffer, 0, sizeof(buffer));
    sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);

    // blocking call to receive data
    auto size = recvfrom(socket_fd, buffer, sizeof(buffer), 0,
                         (sockaddr *)&addr, &addr_len);

    if (size < 0) {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Failed Receive Request. Retrying...");
      continue;
    }

    // Check if the received data is a ping
    if (memcmp(buffer, ping_message, sizeof(*ping_message)) == 0) {
      // Get the remote address and port
      remote_addr = addr;
      remote_address = inet_ntoa(addr.sin_addr);
      remote_port = ntohs(addr.sin_port);
      // Send a pong response to the remote
      // TODO: fix this magic number 10 (size of pong_message) @OJII3
      sendto(socket_fd, pong_message, 10, 0, (sockaddr *)&remote_addr,
             sizeof(remote_addr));
      connection_state = ConnectionState::Connected;
      RCLCPP_INFO_STREAM(this->get_logger(), "Connected to " << remote_address
                                                             << ":"
                                                             << remote_port);
      break;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Invalid Request. Retrying...");
  }

  return;
}

void ROS2MobileController::send(const std::vector<std::byte> &buffer) {
  if (connection_state != ConnectionState::Connected)
    return;
  sendto(socket_fd, buffer.data(), buffer.size(), 0, (sockaddr *)&local_addr,
         sizeof(local_addr));
}

/**
 * @brief Proccess a ros2usb_msgs::msg::USBPacket message and convert it to a
 * std::vector<std::byte> buffer {id, ...packet(including header and footer)}
 */
void ROS2MobileController::rosmsgToBytes(
    const ros2usb_msgs::msg::USBPacket &msg,
    std::vector<std::byte> &result_buffer) {
  auto id = static_cast<std::byte>(msg.id.data);
  memcpy(result_buffer.data(), msg.packet.data.data(), msg.packet.data.size());
  result_buffer.insert(result_buffer.begin(), id);
}

void ROS2MobileController::topicCallback(
    const ros2usb_msgs::msg::USBPacket &msg) {
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Received message from topic " << sub_topic);
  std::vector<std::byte> buffer;
  rosmsgToBytes(msg, buffer);
  this->send(buffer);
}

void ROS2MobileController::Receive(std::array<std::byte, 1024> &buffer) {
  if (connection_state == ConnectionState::Connected) {
    recvfrom(socket_fd, &buffer, sizeof(buffer), 0, (sockaddr *)&remote_addr,
             &remote_addr_len);
  }
}
