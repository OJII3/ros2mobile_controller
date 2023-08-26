#include "ros2mobile_controller.hpp"

using namespace std;
using std::placeholders::_1;

Ros2MobileController::Ros2MobileController() : Node("ros2mobile_controller") {
  subscription_ = this->create_subscription<ros2usb_msgs::msg::USBPacket>(
      sub_topic, 10, bind(&Ros2MobileController::topic_callback, this, _1));
}

Ros2MobileController::~Ros2MobileController() { close(socket_fd); }

bool Ros2MobileController::connectToController() {
  socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  receiver_addr.sin_family = AF_INET;
  receiver_addr.sin_addr.s_addr = inet_addr(local_address.c_str());
  receiver_addr.sin_port = htons(local_port);
  // Bind the receiver socket to the local address and port
  while (rclcpp::ok()) {
    if (bind(socket_fd, (sockaddr *)&receiver_addr, sizeof(receiver_addr)) <
        0) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to bind receiver socket");
      return false;
    }
    RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for connection ...");
    char buffer[1024];
    socklen_t sender_addr_len = sizeof(sender_addr);
    memset(buffer, 0, sizeof(buffer));
    recvfrom(socket_fd, buffer, sizeof(buffer), 0, (sockaddr *)&sender_addr,
             &sender_addr_len);
    if (sizeof(buffer) == 0) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Failed to receive data From "
                              << inet_ntoa(sender_addr.sin_addr) << ":"
                              << ntohs(sender_addr.sin_port));
      rclcpp::sleep_for(chrono::seconds(1));
      continue;
    }

    // Check if the received data is a ping
    // request
    if (memcmp(buffer, "ping-robot", 10) != 0) {
      RCLCPP_INFO_STREAM(this->get_logger(), "Received invalid data");
      rclcpp::sleep_for(chrono::seconds(1));
      continue;
    } else {
      // Get the remote address and port
      remote_address = inet_ntoa(sender_addr.sin_addr);
      remote_port = ntohs(sender_addr.sin_port);
      RCLCPP_INFO_STREAM(this->get_logger(), "Received ping request from "
                                                 << remote_address << ":"
                                                 << remote_port);
      socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
      sender_addr.sin_family = AF_INET;
      sender_addr.sin_addr.s_addr = inet_addr(remote_address.c_str());
      sender_addr.sin_port = htons(remote_port);
      // Send a pong response to the remote
      if (sendto(socket_fd, "pong-robot", 10, 0, (sockaddr *)&sender_addr,
                 sizeof(sender_addr)) < 0) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to send pong response");
        rclcpp::sleep_for(chrono::seconds(1));
        continue;
      }
      RCLCPP_INFO_STREAM(this->get_logger(), "Sent pong response to "
                                                 << remote_address << ":"
                                                 << remote_port);
      break;
    }
  }

  return true;
}

void Ros2MobileController::sendToController(
    const ros2usb_msgs::msg::USBPacket::SharedPtr &msg) {

  auto id = static_cast<byte>(msg->id.data);
  vector<byte> packet;
  memcpy(packet.data(), msg->packet.data.data(), msg->packet.data.size());
  packet.insert(packet.begin(), id);
  sendto(socket_fd, packet.data(), packet.size(), 0, (sockaddr *)&sender_addr,
         sizeof(sender_addr));
}

void Ros2MobileController::topic_callback(
    const ros2usb_msgs::msg::USBPacket &msg) {
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Received message from topic " << sub_topic);
  this->sendToController(std::make_shared<ros2usb_msgs::msg::USBPacket>(msg));
}
