#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

/**
 * @brief Construct a new UDPClient object
 *
 * @param local_ip
 * @param port The port to bind to
 */
class UDPClient {
 public:
  UDPClient &operator=(const UDPClient &) = delete;
  UDPClient(UDPClient &&) = delete;
  UDPClient(const UDPClient &) = default;
  UDPClient &operator=(UDPClient &&) = delete;
  UDPClient(const std::string &local_ip, const uint16_t &port);
  ~UDPClient();

  struct UdpReceiveResult {
    std::vector<uint8_t> data_;
    std::string data_str_;
    std::string remote_ip_;
    uint16_t remote_port_;
  };

  UdpReceiveResult receive(const int &data_size);
  UdpReceiveResult receive();
  void sendto(const std::vector<uint8_t> &data, const std::string &remote_ip,
              const uint16_t &remote_port);
  void sendto(const std::string &data, const std::string &remote_ip,
              const uint16_t &remote_port);
  void broadcast(const std::vector<uint8_t> &data, const uint16_t &remote_port);
  void broadcast(const std::string &data, const uint16_t &remote_port);

 private:
  static constexpr int data_size_default = 1024;
  int socket_fd_;
  sockaddr_in local_addr_;
  sockaddr_in remote_addr_;
};
