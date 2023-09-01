#include <rclcpp/create_timer.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ros2mobile_controller.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ROS2MobileController>();
  constexpr auto period = std::chrono::milliseconds(10);
  auto timer =
      node->create_wall_timer(period, [&]() -> void { node->update(); });
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
