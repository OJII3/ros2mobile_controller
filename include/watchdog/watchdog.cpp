#include "watchdog.hpp"

void Watchdog::startLoop(const std::chrono::milliseconds &interval_ms,
                         const std::chrono::milliseconds &timeout_ms,
                         const std::function<void()> &callback_on_timeout) {
  is_running_ = true;
  while (rclcpp::ok() && is_running_) {
    if (std::chrono::steady_clock::now() - last_update_ > timeout_ms) {
      callback_on_timeout();
    }
    rclcpp::sleep_for(interval_ms);
  }
}

void Watchdog::stopLoop() { is_running_ = false; }
