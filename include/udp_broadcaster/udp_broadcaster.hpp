#pragma once

#include <asio.hpp>
#include <chrono>
#include <ratio>
#include <rclcpp/rclcpp.hpp>
#include <vector>

class UDPBroadcaster : public rclcpp::Node {
 public:
  UDPBroadcaster(const UDPBroadcaster &) = delete;
  UDPBroadcaster(UDPBroadcaster &&) = delete;
  UDPBroadcaster &operator=(const UDPBroadcaster &) = delete;
  UDPBroadcaster &operator=(UDPBroadcaster &&) = delete;
  UDPBroadcaster(const std::string &node_name, const uint16_t &local_send_port,
                 const std::chrono::milliseconds &interval_ms);
  ~UDPBroadcaster() override;
  void updateBuffer(const std::vector<uint8_t> &buffer);
  void startBroadcastLoop(const uint16_t &remote_listen_port);
  void stopLoop();

 private:
  uint16_t local_send_port_;
  asio::io_service io_service_;
  asio::ip::udp::socket socket_;
  bool is_running_ = false;
  std::chrono::milliseconds interval_;
  std::vector<uint8_t> buffer_ = {};
};
