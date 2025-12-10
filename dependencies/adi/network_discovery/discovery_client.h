#pragma once

#include "protocol.h"
#include "network_utils.h"
#include "platform.h"
#include <string>
#include <vector>

#ifndef PLATFORM_WINDOWS
#include <netinet/in.h>
#endif

namespace network_discovery {

struct ServerInfo {
    std::string server_name;
    std::string ip_address;
    uint16_t port;
    NetworkMode mode;
    std::string serial_number;
    struct sockaddr_in address;
};

struct NetworkConfig {
    std::string ip_address;
    std::string netmask;
    std::string gateway;
    NetworkMode mode;
};

class DiscoveryClient {
public:
    explicit DiscoveryClient(uint16_t port = DEFAULT_DISCOVERY_PORT, const std::string& interface = "");
    ~DiscoveryClient();
    
    // Discover servers on the network
    std::vector<ServerInfo> discover_servers(int timeout_seconds = 3);
    
    // Get IP information from a specific server
    bool get_ip_info(const ServerInfo& server, NetworkConfig& config);
    
    // Set DHCP mode on server
    bool set_dhcp(const ServerInfo& server, std::string& response_message);
    
    // Set static IP on server
    bool set_static_ip(const ServerInfo& server,
                      const std::string& ip_address,
                      const std::string& netmask,
                      const std::string& gateway,
                      std::string& response_message);
    
private:
    bool send_request(const MessageHeader& header, 
                     const std::vector<uint8_t>& payload,
                     const struct sockaddr_in* dest_addr = nullptr);
    
    bool receive_response(MessageHeader& header, 
                         std::vector<uint8_t>& payload,
                         int timeout_seconds,
                         struct sockaddr_in* from_addr = nullptr);
    
    uint16_t port_;
    int socket_fd_;
    std::string interface_;
};

} // namespace network_discovery
