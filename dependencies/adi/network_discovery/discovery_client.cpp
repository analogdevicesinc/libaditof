#include "discovery_client.h"
#include "platform.h"
#include <iostream>
#include <cstring>
#include <chrono>
#include <thread>
#include <set>

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

DiscoveryClient::DiscoveryClient(uint16_t port, const std::string& interface)
    : port_(port), socket_fd_(-1), interface_(interface) {
}

DiscoveryClient::~DiscoveryClient() {
    if (socket_fd_ >= 0) {
        close(socket_fd_);
    }
}

std::vector<ServerInfo> DiscoveryClient::discover_servers(int timeout_seconds) {
    std::vector<ServerInfo> servers;
    std::set<std::string> seen_servers; // To avoid duplicates
    
    // Create broadcast socket - bind to port 0 to allow OS to assign a port
    // This is needed on Windows to receive responses
    socket_fd_ = NetworkUtils::create_broadcast_socket(0, true);
    if (socket_fd_ < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return servers;
    }
    
    // Bind to specific interface if specified
    if (!interface_.empty()) {
        if (!NetworkUtils::bind_socket_to_interface(socket_fd_, interface_)) {
            std::cerr << "Warning: Could not bind to interface " << interface_ << std::endl;
            std::cerr << "Continuing with default interface..." << std::endl;
        }
    }
    
    // Send discovery request with staggered timing to maximize chance of catching server
    MessageHeader header(MessageType::DISCOVERY_REQUEST, 0);
    std::cout << "Sending discovery broadcasts on port " << port_ << "..." << std::endl;
    
    auto start_time = std::chrono::steady_clock::now();
    int broadcast_count = 0;
    auto last_broadcast = std::chrono::steady_clock::now() - std::chrono::milliseconds(500);
    
    // Receive responses while periodically re-broadcasting
    while (true) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
        
        if (elapsed >= timeout_seconds) {
            break;
        }
        
        // Send broadcast every 400ms during the first 1.6 seconds (4 broadcasts total)
        // This spreads broadcasts across server's 1-second timeout window
        auto time_since_last_broadcast = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_broadcast).count();
        
        if (broadcast_count < 4 && time_since_last_broadcast >= 400) {
            if (!send_request(header, std::vector<uint8_t>())) {
                std::cerr << "Failed to send discovery request (attempt " << (broadcast_count+1) << ")" << std::endl;
            }
            last_broadcast = now;
            broadcast_count++;
        }
        
        int remaining = timeout_seconds - elapsed;
        if (remaining <= 0) remaining = 1;
        
        MessageHeader resp_header;
        std::vector<uint8_t> payload;
        struct sockaddr_in from_addr;
        
        if (receive_response(resp_header, payload, remaining, &from_addr)) {
            if (resp_header.type == MessageType::DISCOVERY_RESPONSE) {
                DiscoveryResponsePayload resp_payload = 
                    DiscoveryResponsePayload::deserialize(payload.data(), payload.size());
                
                std::string server_key = std::string(resp_payload.ip_address) + ":" +
                                        std::to_string(resp_payload.service_port);
                
                if (seen_servers.find(server_key) == seen_servers.end()) {
                    ServerInfo info;
                    info.server_name = resp_payload.server_name;
                    info.ip_address = resp_payload.ip_address;
                    info.port = resp_payload.service_port;
                    info.mode = resp_payload.current_mode;
                    info.serial_number = resp_payload.serial_number;
                    info.address = from_addr;
                    
                    servers.push_back(info);
                    seen_servers.insert(server_key);
                    
                    std::cout << "Found server: " << info.server_name 
                             << " (SN: " << info.serial_number << ")"
                             << " at " << info.ip_address 
                             << ":" << info.port 
                             << " (mode: " << (info.mode == NetworkMode::DHCP ? "DHCP" : "Static") 
                             << ")" << std::endl;
                }
            }
        }
    }
    
    close(socket_fd_);
    socket_fd_ = -1;
    
    return servers;
}

bool DiscoveryClient::get_ip_info(const ServerInfo& server, NetworkConfig& config) {
    // Create socket
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return false;
    }
    
    // Send get IP request
    MessageHeader header(MessageType::GET_IP_REQUEST, 0);
    if (!send_request(header, std::vector<uint8_t>(), &server.address)) {
        std::cerr << "Failed to send get IP request" << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    
    // Receive response
    MessageHeader resp_header;
    std::vector<uint8_t> payload;
    
    if (!receive_response(resp_header, payload, 5, nullptr)) {
        std::cerr << "Failed to receive response" << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    
    close(socket_fd_);
    socket_fd_ = -1;
    
    if (resp_header.type == MessageType::GET_IP_RESPONSE) {
        GetIPResponsePayload resp_payload = 
            GetIPResponsePayload::deserialize(payload.data(), payload.size());
        
        config.ip_address = resp_payload.ip_address;
        config.netmask = resp_payload.netmask;
        config.gateway = resp_payload.gateway;
        config.mode = resp_payload.mode;
        
        return true;
    } else if (resp_header.type == MessageType::ERROR_RESPONSE) {
        ErrorResponsePayload error_payload = 
            ErrorResponsePayload::deserialize(payload.data(), payload.size());
        std::cerr << "Error from server: " << error_payload.error_message << std::endl;
    }
    
    return false;
}

bool DiscoveryClient::set_dhcp(const ServerInfo& server, std::string& response_message) {
    // Create socket
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return false;
    }
    
    // Send set DHCP request
    MessageHeader header(MessageType::SET_DHCP_REQUEST, 0);
    if (!send_request(header, std::vector<uint8_t>(), &server.address)) {
        std::cerr << "Failed to send set DHCP request" << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    
    // Receive response
    MessageHeader resp_header;
    std::vector<uint8_t> payload;
    
    if (!receive_response(resp_header, payload, 10, nullptr)) {
        std::cerr << "Failed to receive response" << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    
    close(socket_fd_);
    socket_fd_ = -1;
    
    if (resp_header.type == MessageType::CONFIG_RESPONSE) {
        ConfigResponsePayload resp_payload = 
            ConfigResponsePayload::deserialize(payload.data(), payload.size());
        
        response_message = resp_payload.message;
        return resp_payload.status == StatusCode::SUCCESS;
    } else if (resp_header.type == MessageType::ERROR_RESPONSE) {
        ErrorResponsePayload error_payload = 
            ErrorResponsePayload::deserialize(payload.data(), payload.size());
        response_message = error_payload.error_message;
    }
    
    return false;
}

bool DiscoveryClient::set_static_ip(const ServerInfo& server,
                                   const std::string& ip_address,
                                   const std::string& netmask,
                                   const std::string& gateway,
                                   std::string& response_message) {
    // Create socket
    socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        std::cerr << "Failed to create socket" << std::endl;
        return false;
    }
    
    // Prepare payload
    SetStaticIPPayload payload;
    std::strncpy(payload.ip_address, ip_address.c_str(), sizeof(payload.ip_address) - 1);
    std::strncpy(payload.netmask, netmask.c_str(), sizeof(payload.netmask) - 1);
    std::strncpy(payload.gateway, gateway.c_str(), sizeof(payload.gateway) - 1);
    
    // Send set static IP request
    MessageHeader header(MessageType::SET_STATIC_IP_REQUEST, sizeof(SetStaticIPPayload));
    if (!send_request(header, payload.serialize(), &server.address)) {
        std::cerr << "Failed to send set static IP request" << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    
    // Receive response
    MessageHeader resp_header;
    std::vector<uint8_t> resp_payload_data;
    
    if (!receive_response(resp_header, resp_payload_data, 10, nullptr)) {
        std::cerr << "Failed to receive response" << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }
    
    close(socket_fd_);
    socket_fd_ = -1;
    
    if (resp_header.type == MessageType::CONFIG_RESPONSE) {
        ConfigResponsePayload resp_payload = 
            ConfigResponsePayload::deserialize(resp_payload_data.data(), resp_payload_data.size());
        
        response_message = resp_payload.message;
        return resp_payload.status == StatusCode::SUCCESS;
    } else if (resp_header.type == MessageType::ERROR_RESPONSE) {
        ErrorResponsePayload error_payload = 
            ErrorResponsePayload::deserialize(resp_payload_data.data(), resp_payload_data.size());
        response_message = error_payload.error_message;
    }
    
    return false;
}

bool DiscoveryClient::send_request(const MessageHeader& header, 
                                   const std::vector<uint8_t>& payload,
                                   const struct sockaddr_in* dest_addr) {
    auto header_data = header.serialize();
    
    std::vector<uint8_t> message;
    message.insert(message.end(), header_data.begin(), header_data.end());
    message.insert(message.end(), payload.begin(), payload.end());
    
    if (dest_addr) {
        // Send to specific address
#ifdef PLATFORM_WINDOWS
        int sent = sendto(socket_fd_, (const char*)message.data(), (int)message.size(), 0,
                         (struct sockaddr*)dest_addr, sizeof(*dest_addr));
        return sent == static_cast<int>(message.size());
#else
        ssize_t sent = sendto(socket_fd_, message.data(), message.size(), 0,
                            (struct sockaddr*)dest_addr, sizeof(*dest_addr));
        return sent == static_cast<ssize_t>(message.size());
#endif
    } else {
        // Send broadcast
        return NetworkUtils::send_broadcast(socket_fd_, port_, 
                                          message.data(), message.size());
    }
}

bool DiscoveryClient::receive_response(MessageHeader& header, 
                                      std::vector<uint8_t>& payload,
                                      int timeout_seconds,
                                      struct sockaddr_in* from_addr) {
    // Set timeout - use shorter timeout for more responsive checking
    // This allows the loop to check elapsed time and re-broadcast more frequently
    // Note: timeout_seconds parameter kept for API compatibility but not used here
    (void)timeout_seconds; // Suppress unused parameter warning
#ifdef PLATFORM_WINDOWS
    DWORD timeout_ms = 200; // 200ms instead of full timeout_seconds
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, 
               (const char*)&timeout_ms, sizeof(timeout_ms));
#else
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 200000; // 200ms
    setsockopt(socket_fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif
    
    uint8_t buffer[4096];
    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);
    
#ifdef PLATFORM_WINDOWS
    int recv_len = recvfrom(socket_fd_, (char*)buffer, sizeof(buffer), 0,
                           (struct sockaddr*)&addr, &addr_len);
    
    if (recv_len < static_cast<int>(sizeof(MessageHeader))) {
        return false;
    }
#else
    ssize_t recv_len = recvfrom(socket_fd_, buffer, sizeof(buffer), 0,
                               (struct sockaddr*)&addr, &addr_len);
    
    if (recv_len < static_cast<ssize_t>(sizeof(MessageHeader))) {
        return false;
    }
#endif
    
    if (from_addr) {
        *from_addr = addr;
    }
    
    header = MessageHeader::deserialize(buffer, recv_len);
    if (!header.is_valid()) {
        return false;
    }
    
    if (header.payload_length > 0) {
        size_t payload_offset = sizeof(MessageHeader);
#ifdef PLATFORM_WINDOWS
        if (recv_len >= static_cast<int>(payload_offset + header.payload_length)) {
#else
        if (recv_len >= static_cast<ssize_t>(payload_offset + header.payload_length)) {
#endif
            payload.assign(buffer + payload_offset, 
                         buffer + payload_offset + header.payload_length);
        }
    }
    
    return true;
}

} // namespace network_discovery
