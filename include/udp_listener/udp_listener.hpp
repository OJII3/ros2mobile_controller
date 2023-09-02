#pragma once

#include <asio.hpp>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

class UDPListener {
 public:
  UDPListener(const UDPListener &) = delete;
  UDPListener(UDPListener &&) = delete;
  UDPListener &operator=(const UDPListener &) = delete;
  UDPListener &operator=(UDPListener &&) = delete;
  UDPListener(const uint16_t &local_listen_port);
  ~UDPListener();

  struct ReceiveResult {
    std::vector<uint8_t> buffer_;
    asio::ip::udp::endpoint remote_endpoint_;
    std::chrono::steady_clock::time_point receive_time_;
  };

  void startReceiveLoop(
      const size_t &max_data_size,
      const std::function<void(const ReceiveResult &)> &callback);
  void stopLoop();

 private:
  asio::io_service io_service_;
  asio::ip::udp::socket socket_;
  uint16_t local_listen_port_;
  bool is_running_ = false;
};
