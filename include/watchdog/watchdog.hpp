#pragma once

#include <chrono>
#include <rclcpp/node.hpp>
#include <string>

/**
 * @brief Watchdog class
 * @param node_name Name of the node
 */
class Watchdog {
 public:
  /**
   * @brief Start the watchdog, make sure not to provide a blocking callback
   */
  void startLoop(const std::chrono::milliseconds &interval_ms,
                 const std::chrono::milliseconds &timeout_ms,
                 const std::function<void()> &callback_on_timeout);
  void stopLoop();
  bool isRunning() const { return is_running_; }
  /**
   * @brief Update the watchdog
   */
  void update() { last_update_ = std::chrono::steady_clock::now(); }

 private:
  bool is_running_ = false;
  std::chrono::steady_clock::time_point last_update_;
  std::function<bool()> check_updates_;
  std::function<void()> callback_on_timeout_;
};
