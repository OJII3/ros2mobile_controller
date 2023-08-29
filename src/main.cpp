#include "ros2mobile_controller.hpp"
#include <rclcpp/executors.hpp>

using namespace std;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = make_shared<ROS2MobileController>();

  try {
    auto timer = node->create_wall_timer(10ms, [&]() { node->update(); });

    while (rclcpp::ok()) {
      rclcpp::spin_some(node);
    }

  } catch (...) {
    RCLCPP_INFO_STREAM(node->get_logger(), "Exception thrown. Exiting...");
  }
  rclcpp::shutdown();
}
