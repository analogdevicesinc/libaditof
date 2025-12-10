#pragma once

#include "platform.h"
#include <string>
#include <vector>

#ifndef PLATFORM_WINDOWS
#include <netinet/in.h>
#endif

namespace network_discovery {

class NetworkUtils {
public:
    // Get all network interface IP addresses
    static std::vector<std::string> get_local_ip_addresses();
    
    // Get primary network interface IP address
    static std::string get_primary_ip_address();
    
    // Get hostname
    static std::string get_hostname();
    
    // Get network information for a specific interface
    struct NetworkInfo {
        std::string interface_name;
        std::string ip_address;
        std::string netmask;
        std::string gateway;
        bool is_dhcp;
    };
    
    static NetworkInfo get_network_info(const std::string& interface_name = "");
    
    // Get default network interface name
    static std::string get_default_interface();
    
    // Check if running with root/admin privileges
    static bool has_root_privileges();
    
    // Set static IP address (requires root)
    static bool set_static_ip(const std::string& interface_name,
                             const std::string& ip_address,
                             const std::string& netmask,
                             const std::string& gateway);
    
    // Enable DHCP on interface (requires root)
    static bool enable_dhcp(const std::string& interface_name);
    
    // Configure and start DHCP server (requires root)
    static bool start_dhcp_server(const std::string& interface_name,
                                  const std::string& server_ip,
                                  const std::string& range_start,
                                  const std::string& range_end,
                                  const std::string& netmask,
                                  const std::string& gateway);
    
    // Stop DHCP server (requires root)
    static bool stop_dhcp_server(const std::string& interface_name);
    
    // Check if DHCP server is running
    static bool is_dhcp_server_running(const std::string& interface_name);
    
    // Create UDP socket for broadcast
    static int create_broadcast_socket(uint16_t port, bool bind_socket = true);
    
    // Bind socket to specific interface (Linux only, returns true on Windows)
    static bool bind_socket_to_interface(int socket_fd, const std::string& interface);
    
    // Send broadcast message
    static bool send_broadcast(int socket_fd, uint16_t port, 
                              const void* data, size_t len);
    
    // Get broadcast address for interface
    static std::string get_broadcast_address(const std::string& interface_name = "");
};

} // namespace network_discovery
