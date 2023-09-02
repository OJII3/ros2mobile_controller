#include <rclcpp/rclcpp.hpp>

#include "ros2mobile_controller.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ROS2MobileController>();
  try {
    node->start();
    rclcpp::spin(node);
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "Exception in ros2mobile_controller");
  }
  rclcpp::shutdown();
  return 0;
}
