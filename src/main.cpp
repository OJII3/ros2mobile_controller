#include "ros2mobile_controller.hpp"

using namespace std;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = make_shared<Ros2MobileController>();
  try {
    node->connectToController();
    while (rclcpp::ok()) {
      rclcpp::spin(node);
    }
  } catch (...) {
    RCLCPP_ERROR(node->get_logger(), "Error while connecting to controller");
  }
  return 0;
}
