#pragma once

#include <arpa/inet.h>
#include <cstdint>
#include <netinet/in.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <vector>

class UdpClient {
public:
  /**
   * @brief Construct a new UDPClient object
   *
   * @param local_ip
   * @param port The port to bind to
   */
  UdpClient(std::string local_ip, uint16_t port);
  ~UdpClient();

  struct UdpReceiveResult {
    std::vector<uint8_t> data;
    std::string remote_ip;
    uint16_t remote_port;
  };

  UdpReceiveResult receive();
  void sendto(std::vector<uint8_t> &data, std::string remote_ip,
              uint16_t remote_port);
  void broadcast(std::vector<uint8_t> &data, uint16_t remote_port);

private:
  int socket_fd;
  struct sockaddr_in local_addr;
  struct sockaddr_in remote_addr;
};
