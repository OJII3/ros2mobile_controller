#include <future>
#include <rclcpp/logging.hpp>
#include <thread>

#include "udp_broadcaster/udp_broadcaster.hpp"
#include "udp_listener/udp_listener.hpp"
#include "watchdog/watchdog.hpp"

enum class ConnectionStatus { CONNECTED, DISCONNECTED };

int main(int argc, char *argv[]) {
  constexpr int max_data_size = 1024;
  constexpr uint16_t local_listen_port = 10000;
  constexpr uint16_t local_send_port = 5000;  // not important
  constexpr uint16_t remote_listen_port = 10001;
  constexpr std::chrono::milliseconds interval_ms(10);
  constexpr std::chrono::milliseconds timeout_ms(2000);

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto watchdog = std::make_shared<Watchdog>("watchdog");
  auto udp_broadcaster = std::make_shared<UDPBroadcaster>(
      "udp_broadcaster", local_send_port, interval_ms);
  auto udp_listener =
      std::make_shared<UDPListener>("udp_listener", local_listen_port);

  executor.add_node(watchdog);
  executor.add_node(udp_broadcaster);
  executor.add_node(udp_listener);
  RCLCPP_INFO_STREAM(watchdog->get_logger(), "added nodes to executor");

  auto connection_status = ConnectionStatus::DISCONNECTED;

  auto watchdog_async = std::async(std::launch::async, [&]() -> void {
    watchdog->startLoop(interval_ms, timeout_ms, [&]() -> void {
      connection_status = ConnectionStatus::DISCONNECTED;
      RCLCPP_INFO_STREAM(watchdog->get_logger(), "watchdog: timeout");
    });
  });

  RCLCPP_INFO_STREAM(watchdog->get_logger(), "starting watchdog loop");

  auto listen_async = std::async(std::launch::async, [&]() -> void {
    udp_listener->startReceiveLoop(
        max_data_size, [&](const UDPListener::ReceiveResult &result) {
          connection_status = ConnectionStatus::CONNECTED;
          watchdog->update();
          RCLCPP_INFO_STREAM(watchdog->get_logger(),
                             "listen: received " << result.buffer_.size()
                                                 << " bytes from "
                                                 << result.remote_endpoint_);
        });
  });

  RCLCPP_INFO_STREAM(watchdog->get_logger(), "starting receive loop");

  auto broadcast_async = std::async(std::launch::async, [&]() -> void {
    udp_broadcaster->updateBuffer({1, 'S', 'S', 'E', 'E'});
    udp_broadcaster->startBroadcastLoop(remote_listen_port);
  });

  RCLCPP_INFO_STREAM(watchdog->get_logger(), "starting broadcast loop");

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
