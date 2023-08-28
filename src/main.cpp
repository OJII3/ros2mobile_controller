#include "ros2mobile_controller.hpp"

using namespace std;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = make_shared<ROS2MobileController>();

  constexpr int max_connection_failure = 10;
  int connection_failure = 0;

  try {
    node->connect();
    std::array<std::byte, 1024> buffer;
    while (rclcpp::ok()) {
      switch (node->connection_state) {
      case ROS2MobileController::ConnectionState::Disconnected: {
        node->connect();
        break;
      }
      case ROS2MobileController::ConnectionState::Connecting: {
        break;
      }
      case ROS2MobileController::ConnectionState::Connected: {

        // blocking call to receive data
        if (connection_failure >= max_connection_failure) {
          RCLCPP_INFO_STREAM(node->get_logger(), "Connection lost.");
          node->connection_state =
              ROS2MobileController::ConnectionState::Disconnected;
        } else {
          connection_failure = 0;
        }

        // blocking call to receive data
        node->Receive(buffer);
        if (sizeof(buffer) == 0) {
          RCLCPP_INFO_STREAM(node->get_logger(), "Failed Receive Request.");
          connection_failure++;
        } else {
          char *str = reinterpret_cast<char *>(buffer.data());
          RCLCPP_INFO_STREAM(node->get_logger(), "Received data." << *str);
        }
      }
      }
    }

    rclcpp::sleep_for(std::chrono::milliseconds(10));
    rclcpp::spin(node);
  } catch (...) {
    RCLCPP_INFO_STREAM(node->get_logger(), "Exception thrown. Exiting...");
  }
}
