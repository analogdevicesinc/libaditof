#include "discovery_server.h"
#include "platform.h"
#include <iostream>
#include <cstring>
#include <random>
#include <sstream>
#include <iomanip>
#include <fstream>

#ifdef PLATFORM_WINDOWS
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

namespace network_discovery {

DiscoveryServer::DiscoveryServer(uint16_t port, const std::string& serial_number,
                                 const std::string& config_file, const std::string& interface)
    : port_(port), socket_fd_(-1), running_(false), 
      current_mode_(NetworkMode::DHCP), config_file_(config_file) {
    server_name_ = NetworkUtils::get_hostname();
    
    // Get network interface (use specified or default)
    if (!interface.empty()) {
        interface_name_ = interface;
    } else {
        interface_name_ = NetworkUtils::get_default_interface();
    }
    
    update_network_info();
    
    // Use provided serial number or generate random one
    if (!serial_number.empty()) {
        serial_number_ = serial_number;
    } else {
        // Generate random serial number
        std::random_device rd;
        std::mt19937_64 gen(rd());
        std::uniform_int_distribution<uint64_t> dis;
        uint64_t serial = dis(gen);
        
        std::stringstream ss;
        ss << "SN" << std::hex << std::uppercase << std::setfill('0') << std::setw(16) << serial;
        serial_number_ = ss.str();
    }
}

DiscoveryServer::~DiscoveryServer() {
    stop();
}

bool DiscoveryServer::start() {
    if (running_) {
        std::cerr << "Server already running" << std::endl;
        return false;
    }
    
    socket_fd_ = NetworkUtils::create_broadcast_socket(port_, true);
    if (socket_fd_ < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return false;
    }
    
    running_ = true;
    server_thread_ = std::make_unique<std::thread>(&DiscoveryServer::run, this);
    
    std::cout << "Discovery server started on port " << port_ << std::endl;
    std::cout << "Server name: " << server_name_ << std::endl;
    std::cout << "Serial number: " << serial_number_ << std::endl;
    std::cout << "IP address: " << ip_address_ << std::endl;
    std::cout << "Interface: " << interface_name_ << std::endl;
    std::cout << "Mode: " << (current_mode_ == NetworkMode::DHCP ? "DHCP" : "Static") << std::endl;
    
    return true;
}

void DiscoveryServer::stop() {
    if (!running_) {
        return;
    }
    
    running_ = false;
    
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
    
    if (server_thread_ && server_thread_->joinable()) {
        server_thread_->join();
    }
    
    std::cout << "Discovery server stopped" << std::endl;
}

void DiscoveryServer::run() {
    uint8_t buffer[4096];
    struct sockaddr_in client_addr;
    socklen_t addr_len = sizeof(client_addr);
    
    while (running_) {
        // Set timeout for recv
#ifdef PLATFORM_WINDOWS
        DWORD timeout_ms = 1000;
        setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, 
                   (const char*)&timeout_ms, sizeof(timeout_ms));
        
        int recv_len = recvfrom(socket_fd_, (char*)buffer, sizeof(buffer), 0,
                               (struct sockaddr*)&client_addr, &addr_len);
        
        if (recv_len < 0) {
            int err = WSAGetLastError();
            if (err == WSAETIMEDOUT || err == WSAEWOULDBLOCK) {
                continue; // Timeout, check running flag
            }
            std::cerr << "Error receiving data: " << Platform::get_error_string(err) << std::endl;
            continue;
        }
        
        if (recv_len < static_cast<int>(sizeof(MessageHeader))) {
            std::cerr << "Received incomplete message" << std::endl;
            continue;
        }
#else
        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        
        ssize_t recv_len = recvfrom(socket_fd_, buffer, sizeof(buffer), 0,
                                    (struct sockaddr*)&client_addr, &addr_len);
        
        if (recv_len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue; // Timeout, check running flag
            }
            std::cerr << "Error receiving data: " << strerror(errno) << std::endl;
            continue;
        }
        
        if (recv_len < static_cast<ssize_t>(sizeof(MessageHeader))) {
            std::cerr << "Received incomplete message" << std::endl;
            continue;
        }
#endif
        
        // Parse header
        MessageHeader header = MessageHeader::deserialize(buffer, recv_len);
        if (!header.is_valid()) {
            std::cerr << "Received invalid message header" << std::endl;
            continue;
        }
        
        std::cout << "Received request from " << inet_ntoa(client_addr.sin_addr) 
                  << ":" << ntohs(client_addr.sin_port) << std::endl;
        
        // Handle different message types
        switch (header.type) {
            case MessageType::DISCOVERY_REQUEST:
                std::cout << "Handling discovery request" << std::endl;
                handle_discovery_request(client_addr);
                break;
                
            case MessageType::GET_IP_REQUEST:
                std::cout << "Handling get IP request" << std::endl;
                handle_get_ip_request(client_addr);
                break;
                
            case MessageType::SET_DHCP_REQUEST:
                std::cout << "Handling set DHCP request" << std::endl;
                handle_set_dhcp_request(client_addr);
                break;
                
            case MessageType::SET_STATIC_IP_REQUEST: {
                std::cout << "Handling set static IP request" << std::endl;
#ifdef PLATFORM_WINDOWS
                if (recv_len >= static_cast<int>(sizeof(MessageHeader) + sizeof(SetStaticIPPayload))) {
#else
                if (recv_len >= static_cast<ssize_t>(sizeof(MessageHeader) + sizeof(SetStaticIPPayload))) {
#endif
                    SetStaticIPPayload payload = SetStaticIPPayload::deserialize(
                        buffer + sizeof(MessageHeader), 
                        recv_len - sizeof(MessageHeader));
                    handle_set_static_ip_request(client_addr, payload);
                } else {
                    send_error_response(client_addr, StatusCode::ERROR_INVALID_REQUEST,
                                      "Invalid payload size");
                }
                break;
            }
            
            case MessageType::SET_DHCP_SERVER_REQUEST: {
                std::cout << "Handling set DHCP server request" << std::endl;
#ifdef PLATFORM_WINDOWS
                if (recv_len >= static_cast<int>(sizeof(MessageHeader) + sizeof(SetDHCPServerPayload))) {
#else
                if (recv_len >= static_cast<ssize_t>(sizeof(MessageHeader) + sizeof(SetDHCPServerPayload))) {
#endif
                    SetDHCPServerPayload payload = SetDHCPServerPayload::deserialize(
                        buffer + sizeof(MessageHeader),
                        recv_len - sizeof(MessageHeader));
                    handle_set_dhcp_server_request(client_addr, payload);
                } else {
                    send_error_response(client_addr, StatusCode::ERROR_INVALID_REQUEST,
                                      "Invalid payload size");
                }
                break;
            }
                
            default:
                std::cerr << "Unknown message type: " 
                         << static_cast<int>(header.type) << std::endl;
                send_error_response(client_addr, StatusCode::ERROR_INVALID_REQUEST,
                                  "Unknown message type");
                break;
        }
    }
}

void DiscoveryServer::handle_discovery_request(const struct sockaddr_in& client_addr) {
    send_discovery_response(client_addr);
}

void DiscoveryServer::handle_get_ip_request(const struct sockaddr_in& client_addr) {
    update_network_info();
    send_get_ip_response(client_addr);
}

void DiscoveryServer::handle_set_dhcp_request(const struct sockaddr_in& client_addr) {
    if (!NetworkUtils::has_root_privileges()) {
        send_config_response(client_addr, StatusCode::ERROR_PERMISSION_DENIED,
                           "Root privileges required to change network configuration");
        return;
    }
    
    std::cout << "Enabling DHCP on interface " << interface_name_ << std::endl;
    
    if (NetworkUtils::enable_dhcp(interface_name_)) {
        current_mode_ = NetworkMode::DHCP;
        // Wait a bit for DHCP to acquire address
        sleep(2);
        update_network_info();
        
        // Save to config file if configured
        save_config_file();
        
        send_config_response(client_addr, StatusCode::SUCCESS,
                           "DHCP enabled successfully");
        std::cout << "DHCP enabled. New IP: " << ip_address_ << std::endl;
    } else {
        send_config_response(client_addr, StatusCode::ERROR_NETWORK_ERROR,
                           "Failed to enable DHCP");
        std::cerr << "Failed to enable DHCP" << std::endl;
    }
}

void DiscoveryServer::handle_set_static_ip_request(const struct sockaddr_in& client_addr,
                                                   const SetStaticIPPayload& payload) {
    if (!NetworkUtils::has_root_privileges()) {
        send_config_response(client_addr, StatusCode::ERROR_PERMISSION_DENIED,
                           "Root privileges required to change network configuration");
        return;
    }
    
    std::string ip(payload.ip_address);
    std::string netmask(payload.netmask);
    std::string gateway(payload.gateway);
    
    std::cout << "Setting static IP on interface " << interface_name_ << std::endl;
    std::cout << "  IP: " << ip << std::endl;
    std::cout << "  Netmask: " << netmask << std::endl;
    std::cout << "  Gateway: " << gateway << std::endl;
    
    // Stop DHCP server if running
    NetworkUtils::stop_dhcp_server(interface_name_);
    
    if (NetworkUtils::set_static_ip(interface_name_, ip, netmask, gateway)) {
        current_mode_ = NetworkMode::STATIC;
        update_network_info();
        
        // Save to config file if configured
        save_config_file();
        
        send_config_response(client_addr, StatusCode::SUCCESS,
                           "Static IP configured successfully");
        std::cout << "Static IP configured successfully" << std::endl;
    } else {
        send_config_response(client_addr, StatusCode::ERROR_NETWORK_ERROR,
                           "Failed to set static IP");
        std::cerr << "Failed to set static IP" << std::endl;
    }
}

void DiscoveryServer::handle_set_dhcp_server_request(const struct sockaddr_in& client_addr,
                                                     const SetDHCPServerPayload& payload) {
    if (!NetworkUtils::has_root_privileges()) {
        send_config_response(client_addr, StatusCode::ERROR_PERMISSION_DENIED,
                           "Root privileges required to start DHCP server");
        return;
    }
    
    std::string server_ip(payload.server_ip);
    std::string range_start(payload.range_start);
    std::string range_end(payload.range_end);
    std::string netmask(payload.netmask);
    std::string gateway(payload.gateway);
    
    std::cout << "Starting DHCP server on interface " << interface_name_ << std::endl;
    std::cout << "  Server IP: " << server_ip << std::endl;
    std::cout << "  DHCP Range: " << range_start << " - " << range_end << std::endl;
    std::cout << "  Netmask: " << netmask << std::endl;
    std::cout << "  Gateway: " << gateway << std::endl;
    
    if (NetworkUtils::start_dhcp_server(interface_name_, server_ip, range_start, 
                                        range_end, netmask, gateway)) {
        current_mode_ = NetworkMode::DHCP_SERVER;
        dhcp_range_start_ = range_start;
        dhcp_range_end_ = range_end;
        update_network_info();
        
        // Save to config file if configured
        save_config_file();
        
        send_config_response(client_addr, StatusCode::SUCCESS,
                           "DHCP server started successfully");
        std::cout << "DHCP server started successfully" << std::endl;
    } else {
        send_config_response(client_addr, StatusCode::ERROR_NETWORK_ERROR,
                           "Failed to start DHCP server");
        std::cerr << "Failed to start DHCP server" << std::endl;
    }
}

void DiscoveryServer::send_discovery_response(const struct sockaddr_in& client_addr) {
    DiscoveryResponsePayload payload;
    std::strncpy(payload.server_name, server_name_.c_str(), sizeof(payload.server_name) - 1);
    std::strncpy(payload.ip_address, ip_address_.c_str(), sizeof(payload.ip_address) - 1);
    std::strncpy(payload.serial_number, serial_number_.c_str(), sizeof(payload.serial_number) - 1);
    payload.service_port = port_;
    payload.current_mode = current_mode_;
    
    MessageHeader header(MessageType::DISCOVERY_RESPONSE, sizeof(DiscoveryResponsePayload));
    
    auto header_data = header.serialize();
    auto payload_data = payload.serialize();
    
    std::vector<uint8_t> response;
    response.insert(response.end(), header_data.begin(), header_data.end());
    response.insert(response.end(), payload_data.begin(), payload_data.end());
    
#ifdef PLATFORM_WINDOWS
    sendto(socket_fd_, (const char*)response.data(), (int)response.size(), 0,
           (struct sockaddr*)&client_addr, sizeof(client_addr));
#else
    sendto(socket_fd_, response.data(), response.size(), 0,
           (struct sockaddr*)&client_addr, sizeof(client_addr));
#endif
    
    std::cout << "Sent discovery response to " << inet_ntoa(client_addr.sin_addr) << std::endl;
}

void DiscoveryServer::send_get_ip_response(const struct sockaddr_in& client_addr) {
    GetIPResponsePayload payload;
    std::strncpy(payload.ip_address, ip_address_.c_str(), sizeof(payload.ip_address) - 1);
    std::strncpy(payload.netmask, netmask_.c_str(), sizeof(payload.netmask) - 1);
    std::strncpy(payload.gateway, gateway_.c_str(), sizeof(payload.gateway) - 1);
    payload.mode = current_mode_;
    
    MessageHeader header(MessageType::GET_IP_RESPONSE, sizeof(GetIPResponsePayload));
    
    auto header_data = header.serialize();
    auto payload_data = payload.serialize();
    
    std::vector<uint8_t> response;
    response.insert(response.end(), header_data.begin(), header_data.end());
    response.insert(response.end(), payload_data.begin(), payload_data.end());
    
#ifdef PLATFORM_WINDOWS
    sendto(socket_fd_, (const char*)response.data(), (int)response.size(), 0,
           (struct sockaddr*)&client_addr, sizeof(client_addr));
#else
    sendto(socket_fd_, response.data(), response.size(), 0,
           (struct sockaddr*)&client_addr, sizeof(client_addr));
#endif
    
    std::cout << "Sent IP info response to " << inet_ntoa(client_addr.sin_addr) << std::endl;
}

void DiscoveryServer::send_config_response(const struct sockaddr_in& client_addr,
                                          StatusCode status, const std::string& message) {
    ConfigResponsePayload payload(status, message);
    
    MessageHeader header(MessageType::CONFIG_RESPONSE, sizeof(ConfigResponsePayload));
    
    auto header_data = header.serialize();
    auto payload_data = payload.serialize();
    
    std::vector<uint8_t> response;
    response.insert(response.end(), header_data.begin(), header_data.end());
    response.insert(response.end(), payload_data.begin(), payload_data.end());
    
#ifdef PLATFORM_WINDOWS
    sendto(socket_fd_, (const char*)response.data(), (int)response.size(), 0,
           (struct sockaddr*)&client_addr, sizeof(client_addr));
#else
    sendto(socket_fd_, response.data(), response.size(), 0,
           (struct sockaddr*)&client_addr, sizeof(client_addr));
#endif
    
    std::cout << "Sent config response to " << inet_ntoa(client_addr.sin_addr) << std::endl;
}

void DiscoveryServer::send_error_response(const struct sockaddr_in& client_addr,
                                         StatusCode error_code, const std::string& message) {
    ErrorResponsePayload payload(error_code, message);
    
    MessageHeader header(MessageType::ERROR_RESPONSE, sizeof(ErrorResponsePayload));
    
    auto header_data = header.serialize();
    auto payload_data = payload.serialize();
    
    std::vector<uint8_t> response;
    response.insert(response.end(), header_data.begin(), header_data.end());
    response.insert(response.end(), payload_data.begin(), payload_data.end());
    
#ifdef PLATFORM_WINDOWS
    sendto(socket_fd_, (const char*)response.data(), (int)response.size(), 0,
           (struct sockaddr*)&client_addr, sizeof(client_addr));
#else
    sendto(socket_fd_, response.data(), response.size(), 0,
           (struct sockaddr*)&client_addr, sizeof(client_addr));
#endif
    
    std::cout << "Sent error response to " << inet_ntoa(client_addr.sin_addr) << std::endl;
}

void DiscoveryServer::update_network_info() {
    auto info = NetworkUtils::get_network_info(interface_name_);
    ip_address_ = info.ip_address.empty() ? NetworkUtils::get_primary_ip_address() : info.ip_address;
    netmask_ = info.netmask;
    gateway_ = info.gateway;
    
    if (info.is_dhcp) {
        current_mode_ = NetworkMode::DHCP;
    }
    // Note: We keep current_mode_ as is if not detected as DHCP,
    // since we may have just set it to STATIC
}

void DiscoveryServer::apply_network_config(NetworkMode mode, const NetworkConfig& config) {
    if (!NetworkUtils::has_root_privileges()) {
        std::cout << "Warning: No root privileges, skipping network configuration" << std::endl;
        return;
    }
    
    if (mode == NetworkMode::DHCP) {
        std::cout << "Applying DHCP configuration from config file..." << std::endl;
        if (NetworkUtils::enable_dhcp(interface_name_)) {
            current_mode_ = NetworkMode::DHCP;
            sleep(2); // Wait for DHCP
            update_network_info();
            std::cout << "DHCP configured. IP: " << ip_address_ << std::endl;
        } else {
            std::cerr << "Failed to apply DHCP configuration" << std::endl;
        }
    } else if (mode == NetworkMode::STATIC) {
        std::cout << "Applying static IP configuration from config file..." << std::endl;
        std::cout << "  IP: " << config.ip_address << std::endl;
        std::cout << "  Netmask: " << config.netmask << std::endl;
        std::cout << "  Gateway: " << config.gateway << std::endl;
        
        if (NetworkUtils::set_static_ip(interface_name_, config.ip_address, 
                                        config.netmask, config.gateway)) {
            current_mode_ = NetworkMode::STATIC;
            update_network_info();
            std::cout << "Static IP configured successfully" << std::endl;
        } else {
            std::cerr << "Failed to apply static IP configuration" << std::endl;
        }
    } else if (mode == NetworkMode::DHCP_SERVER) {
        std::cout << "Applying DHCP server configuration from config file..." << std::endl;
        std::cout << "  Server IP: " << config.ip_address << std::endl;
        std::cout << "  Range: " << config.dhcp_range_start << " - " << config.dhcp_range_end << std::endl;
        std::cout << "  Netmask: " << config.netmask << std::endl;
        std::cout << "  Gateway: " << config.gateway << std::endl;
        
        // First set static IP for the server interface
        if (!NetworkUtils::set_static_ip(interface_name_, config.ip_address, 
                                         config.netmask, config.gateway)) {
            std::cerr << "Failed to configure server interface with static IP" << std::endl;
            return;
        }
        
        // Then start the DHCP server
        if (NetworkUtils::start_dhcp_server(interface_name_, config.ip_address,
                                            config.dhcp_range_start, config.dhcp_range_end,
                                            config.netmask, config.gateway)) {
            current_mode_ = NetworkMode::DHCP_SERVER;
            dhcp_range_start_ = config.dhcp_range_start;
            dhcp_range_end_ = config.dhcp_range_end;
            update_network_info();
            std::cout << "DHCP server started successfully" << std::endl;
        } else {
            std::cerr << "Failed to start DHCP server" << std::endl;
        }
    }
}

void DiscoveryServer::save_config_file() {
    if (config_file_.empty()) {
        return; // No config file specified
    }
    
    std::cout << "Saving configuration to " << config_file_ << std::endl;
    
    std::ofstream file(config_file_);
    if (!file.is_open()) {
        std::cerr << "Warning: Could not open config file for writing: " << config_file_ << std::endl;
        return;
    }
    
    // Write JSON config
    file << "{" << std::endl;
    file << "  \"serial_number\": \"" << serial_number_ << "\"," << std::endl;
    file << "  \"interface\": \"" << interface_name_ << "\"," << std::endl;
    
    std::string mode_str;
    if (current_mode_ == NetworkMode::DHCP) {
        mode_str = "dhcp";
    } else if (current_mode_ == NetworkMode::STATIC) {
        mode_str = "static";
    } else {
        mode_str = "dhcp_server";
    }
    
    file << "  \"network_mode\": \"" << mode_str << "\"," << std::endl;
    file << "  \"static_ip\": {" << std::endl;
    file << "    \"ip_address\": \"" << ip_address_ << "\"," << std::endl;
    file << "    \"netmask\": \"" << netmask_ << "\"," << std::endl;
    file << "    \"gateway\": \"" << gateway_ << "\"" << std::endl;
    file << "  }";
    
    if (current_mode_ == NetworkMode::DHCP_SERVER) {
        file << "," << std::endl;
        file << "  \"dhcp_server\": {" << std::endl;
        file << "    \"range_start\": \"" << dhcp_range_start_ << "\"," << std::endl;
        file << "    \"range_end\": \"" << dhcp_range_end_ << "\"" << std::endl;
        file << "  }" << std::endl;
    } else {
        file << std::endl;
    }
    
    file << "}" << std::endl;
    
    file.close();
    std::cout << "Configuration saved successfully" << std::endl;
}

} // namespace network_discovery
