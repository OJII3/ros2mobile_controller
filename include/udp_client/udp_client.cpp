#include "udp_client.hpp"

UdpClient::UdpClient(std::string local_ip, uint16_t port) {
  socket_fd = socket(AF_INET, SOCK_DGRAM, 0);
  local_addr.sin_family = AF_INET;
  local_addr.sin_addr.s_addr = inet_addr(local_ip.c_str());
  local_addr.sin_port = htons(port);

  // bind socket to port
  if (bind(socket_fd, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0)
    throw std::runtime_error("Failed to bind socket");
}

UdpClient::~UdpClient() { close(socket_fd); }

UdpClient::UdpReceiveResult UdpClient::receive() {
  std::vector<uint8_t> data(1024);
  socklen_t addr_len = sizeof(remote_addr);
  int len = recvfrom(socket_fd, data.data(), data.size(), 0,
                     (struct sockaddr *)&remote_addr, &addr_len);
  data.resize(len);
  return {data, inet_ntoa(remote_addr.sin_addr), ntohs(remote_addr.sin_port)};
}

void UdpClient::sendto(std::vector<uint8_t> &data, std::string remote_ip,
                       uint16_t remote_port) {
  remote_addr.sin_family = AF_INET;
  remote_addr.sin_addr.s_addr = inet_addr(remote_ip.c_str());
  remote_addr.sin_port = htons(remote_port);

  ::sendto(socket_fd, data.data(), data.size(), 0,
           (struct sockaddr *)&remote_addr, sizeof(remote_addr));
}

void UdpClient::broadcast(std::vector<uint8_t> &data, uint16_t remote_port) {
  remote_addr.sin_family = AF_INET;
  remote_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
  remote_addr.sin_port = htons(remote_port);

  ::sendto(socket_fd, data.data(), data.size(), 0,
           (struct sockaddr *)&remote_addr, sizeof(remote_addr));
}
