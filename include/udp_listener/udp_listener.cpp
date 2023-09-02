#include "udp_listener.hpp"

UDPListener::UDPListener(const uint16_t &local_listen_port)
    : socket_(io_service_,
              asio::ip::udp::endpoint(asio::ip::udp::v4(), local_listen_port)),
      local_listen_port_(local_listen_port) {}

UDPListener::~UDPListener() {
  stopLoop();
  io_service_.stop();
  socket_.close();
  socket_.shutdown(asio::ip::udp::socket::shutdown_both);
}

void UDPListener::startReceiveLoop(
    const size_t &max_data_size,
    const std::function<void(const ReceiveResult &)> &callback) {
  is_running_ = true;
  ReceiveResult result;

  while (rclcpp::ok() && is_running_) {
    std::vector<uint8_t> buffer(max_data_size);
    asio::ip::udp::endpoint remote_endpoint;
    size_t len = socket_.receive_from(asio::buffer(buffer), remote_endpoint);
    buffer.resize(len);
    result.buffer_ = buffer;
    result.remote_endpoint_ = remote_endpoint;
    result.receive_time_ = std::chrono::steady_clock::now();
    callback(result);
  }
}

void UDPListener::stopLoop() { is_running_ = false; }
