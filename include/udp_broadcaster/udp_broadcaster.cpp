#include "udp_broadcaster.hpp"

UDPBroadcaster::UDPBroadcaster(const std::string &node_name,
                               const uint16_t &local_send_port,
                               const std::chrono::milliseconds &interval_ms)
    : rclcpp::Node(node_name),
      local_send_port_(local_send_port),
      socket_(io_service_,
              asio::ip::udp::endpoint(asio::ip::udp::v4(), local_send_port)),
      interval_(interval_ms) {
  socket_.set_option(asio::socket_base::broadcast(true));
}

UDPBroadcaster::~UDPBroadcaster() {
  stopLoop();
  io_service_.stop();
  socket_.close();
}

void UDPBroadcaster::startBroadcastLoop(const uint16_t &remote_listen_port) {
  is_running_ = true;
  while (rclcpp::ok() && is_running_) {
    socket_.send_to(asio::buffer(buffer_),
                    asio::ip::udp::endpoint(asio::ip::address_v4::broadcast(),
                                            remote_listen_port));
    rclcpp::sleep_for(interval_);
  }
}

void UDPBroadcaster::stopLoop() { is_running_ = false; }

void UDPBroadcaster::updateBuffer(const std::vector<uint8_t> &buffer) {
  buffer_.clear();
  buffer_.resize(buffer.size());
  memcpy(buffer_.data(), buffer.data(), buffer.size());
}
