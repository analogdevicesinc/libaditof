#pragma once

#include "protocol.h"
#include "network_utils.h"
#include <string>
#include <atomic>
#include <thread>
#include <memory>

namespace network_discovery {

struct NetworkConfig {
    std::string ip_address;
    std::string netmask;
    std::string gateway;
    // DHCP server specific
    std::string dhcp_range_start;
    std::string dhcp_range_end;
};

class DiscoveryServer {
public:
    explicit DiscoveryServer(uint16_t port = DEFAULT_DISCOVERY_PORT, 
                            const std::string& serial_number = "",
                            const std::string& config_file = "",
                            const std::string& interface = "");
    ~DiscoveryServer();
    
    // Start the server
    bool start();
    
    // Stop the server
    void stop();
    
    // Check if server is running
    bool is_running() const { return running_; }
    
    // Get current network mode
    NetworkMode get_network_mode() const { return current_mode_; }
    
    // Get server info
    std::string get_server_name() const { return server_name_; }
    std::string get_ip_address() const { return ip_address_; }
    uint16_t get_port() const { return port_; }
    
private:
    void run();
    void handle_discovery_request(const struct sockaddr_in& client_addr);
    void handle_get_ip_request(const struct sockaddr_in& client_addr);
    void handle_set_dhcp_request(const struct sockaddr_in& client_addr);
    void handle_set_static_ip_request(const struct sockaddr_in& client_addr,
                                     const SetStaticIPPayload& payload);
    void handle_set_dhcp_server_request(const struct sockaddr_in& client_addr,
                                        const SetDHCPServerPayload& payload);
    
    void send_discovery_response(const struct sockaddr_in& client_addr);
    void send_get_ip_response(const struct sockaddr_in& client_addr);
    void send_config_response(const struct sockaddr_in& client_addr,
                             StatusCode status, const std::string& message);
    void send_error_response(const struct sockaddr_in& client_addr,
                           StatusCode error_code, const std::string& message);
    
    void update_network_info();
    
    uint16_t port_;
    int socket_fd_;
    std::atomic<bool> running_;
    std::unique_ptr<std::thread> server_thread_;
    
    std::string server_name_;
    std::string ip_address_;
    std::string interface_name_;
    NetworkMode current_mode_;
    std::string serial_number_;
    std::string config_file_;
    
    // Network info cache
    std::string netmask_;
    std::string gateway_;
    
    // DHCP server config cache
    std::string dhcp_range_start_;
    std::string dhcp_range_end_;
    
public:
    // Helper methods
    void apply_network_config(NetworkMode mode, const NetworkConfig& config);
    
private:
    void save_config_file();
};

} // namespace network_discovery
