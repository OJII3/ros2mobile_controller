#include "udp_client.hpp"

UDPClient::UDPClient(const std::string &local_ip, const uint16_t &port)
    : socket_fd_(socket(AF_INET, SOCK_DGRAM, 0)),
      local_addr_(),
      remote_addr_() {
  local_addr_.sin_family = AF_INET;
  local_addr_.sin_addr.s_addr = inet_addr(local_ip.c_str());
  local_addr_.sin_port = htons(port);

  // bind socket to port
  if (bind(socket_fd_,
           static_cast<sockaddr *>(static_cast<void *>(&local_addr_)),
           sizeof(local_addr_)) < 0)
    throw std::runtime_error("Failed to bind socket");
}

UDPClient::~UDPClient() { close(socket_fd_); }

/**
 * @brief Receive data from socket
 * @return UdpReceiveResult
 */
UDPClient::UdpReceiveResult UDPClient::receive(const int &data_size) {
  std::vector<uint8_t> data(data_size);
  socklen_t addr_len = sizeof(remote_addr_);
  auto len = recvfrom(
      socket_fd_, data.data(), data.size(), 0,
      static_cast<sockaddr *>(static_cast<void *>(&local_addr_)), &addr_len);
  data.resize(len);

  return {data, std::string(data.begin(), data.end()),
          std::string(inet_ntoa(remote_addr_.sin_addr)),
          ntohs(remote_addr_.sin_port)};
}

UDPClient::UdpReceiveResult UDPClient::receive() {
  return receive(data_size_default);
}

void UDPClient::sendto(const std::vector<uint8_t> &data,
                       const std::string &remote_ip,
                       const uint16_t &remote_port) {
  remote_addr_.sin_family = AF_INET;
  remote_addr_.sin_addr.s_addr = inet_addr(remote_ip.c_str());
  remote_addr_.sin_port = htons(remote_port);

  ::sendto(socket_fd_, data.data(), data.size(), 0,
           static_cast<sockaddr *>(static_cast<void *>(&remote_addr_)),
           sizeof(remote_addr_));
}

void UDPClient::sendto(const std::string &data, const std::string &remote_ip,
                       const uint16_t &remote_port) {
  remote_addr_.sin_family = AF_INET;
  remote_addr_.sin_addr.s_addr = inet_addr(remote_ip.c_str());
  remote_addr_.sin_port = htons(remote_port);

  ::sendto(socket_fd_, data.c_str(), data.size(), 0,
           static_cast<sockaddr *>(static_cast<void *>(&remote_addr_)),
           sizeof(remote_addr_));
}

void UDPClient::broadcast(const std::vector<uint8_t> &data,
                          const uint16_t &remote_port) {
  remote_addr_.sin_family = AF_INET;
  remote_addr_.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  remote_addr_.sin_port = htons(remote_port);

  ::sendto(socket_fd_, data.data(), data.size(), 0,
           static_cast<sockaddr *>(static_cast<void *>(&remote_addr_)),
           sizeof(remote_addr_));
}

void UDPClient::broadcast(const std::string &data,
                          const uint16_t &remote_port) {
  remote_addr_.sin_family = AF_INET;
  remote_addr_.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  remote_addr_.sin_port = htons(remote_port);

  ::sendto(socket_fd_, data.c_str(), data.size(), 0,
           static_cast<sockaddr *>(static_cast<void *>(&remote_addr_)),
           sizeof(remote_addr_));
}
