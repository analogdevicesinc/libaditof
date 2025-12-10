#include "protocol.h"
#include "platform.h"
#include <cstring>

#ifndef PLATFORM_WINDOWS
#include <arpa/inet.h>
#endif

namespace network_discovery {

// MessageHeader implementation
MessageHeader::MessageHeader() 
    : magic(PROTOCOL_MAGIC), version(PROTOCOL_VERSION), 
      type(MessageType::DISCOVERY_REQUEST), reserved(0), payload_length(0) {}

MessageHeader::MessageHeader(MessageType msg_type, uint32_t payload_len)
    : magic(PROTOCOL_MAGIC), version(PROTOCOL_VERSION), 
      type(msg_type), reserved(0), payload_length(payload_len) {}

bool MessageHeader::is_valid() const {
    return magic == PROTOCOL_MAGIC && version == PROTOCOL_VERSION;
}

std::vector<uint8_t> MessageHeader::serialize() const {
    std::vector<uint8_t> data(sizeof(MessageHeader));
    uint8_t* ptr = data.data();
    
    uint32_t net_magic = htonl(magic);
    uint16_t net_version = htons(version);
    uint32_t net_payload_length = htonl(payload_length);
    
    std::memcpy(ptr, &net_magic, sizeof(net_magic));
    ptr += sizeof(net_magic);
    std::memcpy(ptr, &net_version, sizeof(net_version));
    ptr += sizeof(net_version);
    std::memcpy(ptr, &type, sizeof(type));
    ptr += sizeof(type);
    std::memcpy(ptr, &reserved, sizeof(reserved));
    ptr += sizeof(reserved);
    std::memcpy(ptr, &net_payload_length, sizeof(net_payload_length));
    
    return data;
}

MessageHeader MessageHeader::deserialize(const uint8_t* data, size_t len) {
    MessageHeader header;
    if (len < sizeof(MessageHeader)) {
        header.magic = 0; // Invalid
        return header;
    }
    
    const uint8_t* ptr = data;
    
    uint32_t net_magic;
    uint16_t net_version;
    uint32_t net_payload_length;
    
    std::memcpy(&net_magic, ptr, sizeof(net_magic));
    header.magic = ntohl(net_magic);
    ptr += sizeof(net_magic);
    
    std::memcpy(&net_version, ptr, sizeof(net_version));
    header.version = ntohs(net_version);
    ptr += sizeof(net_version);
    
    std::memcpy(&header.type, ptr, sizeof(header.type));
    ptr += sizeof(header.type);
    
    std::memcpy(&header.reserved, ptr, sizeof(header.reserved));
    ptr += sizeof(header.reserved);
    
    std::memcpy(&net_payload_length, ptr, sizeof(net_payload_length));
    header.payload_length = ntohl(net_payload_length);
    
    return header;
}

// DiscoveryResponsePayload implementation
DiscoveryResponsePayload::DiscoveryResponsePayload() 
    : service_port(0), current_mode(NetworkMode::DHCP), reserved(0) {
    std::memset(server_name, 0, sizeof(server_name));
    std::memset(ip_address, 0, sizeof(ip_address));
    std::memset(serial_number, 0, sizeof(serial_number));
}

std::vector<uint8_t> DiscoveryResponsePayload::serialize() const {
    std::vector<uint8_t> data(sizeof(DiscoveryResponsePayload));
    uint8_t* ptr = data.data();
    
    std::memcpy(ptr, server_name, sizeof(server_name));
    ptr += sizeof(server_name);
    std::memcpy(ptr, ip_address, sizeof(ip_address));
    ptr += sizeof(ip_address);
    
    uint16_t net_port = htons(service_port);
    std::memcpy(ptr, &net_port, sizeof(net_port));
    ptr += sizeof(net_port);
    
    std::memcpy(ptr, &current_mode, sizeof(current_mode));
    ptr += sizeof(current_mode);
    std::memcpy(ptr, &reserved, sizeof(reserved));
    ptr += sizeof(reserved);
    std::memcpy(ptr, serial_number, sizeof(serial_number));
    
    return data;
}

DiscoveryResponsePayload DiscoveryResponsePayload::deserialize(const uint8_t* data, size_t len) {
    DiscoveryResponsePayload payload;
    if (len < sizeof(DiscoveryResponsePayload)) {
        return payload;
    }
    
    const uint8_t* ptr = data;
    
    std::memcpy(payload.server_name, ptr, sizeof(payload.server_name));
    ptr += sizeof(payload.server_name);
    std::memcpy(payload.ip_address, ptr, sizeof(payload.ip_address));
    ptr += sizeof(payload.ip_address);
    
    uint16_t net_port;
    std::memcpy(&net_port, ptr, sizeof(net_port));
    payload.service_port = ntohs(net_port);
    ptr += sizeof(net_port);
    
    std::memcpy(&payload.current_mode, ptr, sizeof(payload.current_mode));
    ptr += sizeof(payload.current_mode);
    std::memcpy(&payload.reserved, ptr, sizeof(payload.reserved));
    ptr += sizeof(payload.reserved);
    std::memcpy(payload.serial_number, ptr, sizeof(payload.serial_number));
    
    return payload;
}

// GetIPResponsePayload implementation
GetIPResponsePayload::GetIPResponsePayload() 
    : mode(NetworkMode::DHCP) {
    std::memset(ip_address, 0, sizeof(ip_address));
    std::memset(netmask, 0, sizeof(netmask));
    std::memset(gateway, 0, sizeof(gateway));
    std::memset(reserved, 0, sizeof(reserved));
}

std::vector<uint8_t> GetIPResponsePayload::serialize() const {
    std::vector<uint8_t> data(sizeof(GetIPResponsePayload));
    uint8_t* ptr = data.data();
    
    std::memcpy(ptr, ip_address, sizeof(ip_address));
    ptr += sizeof(ip_address);
    std::memcpy(ptr, netmask, sizeof(netmask));
    ptr += sizeof(netmask);
    std::memcpy(ptr, gateway, sizeof(gateway));
    ptr += sizeof(gateway);
    std::memcpy(ptr, &mode, sizeof(mode));
    ptr += sizeof(mode);
    std::memcpy(ptr, reserved, sizeof(reserved));
    
    return data;
}

GetIPResponsePayload GetIPResponsePayload::deserialize(const uint8_t* data, size_t len) {
    GetIPResponsePayload payload;
    if (len < sizeof(GetIPResponsePayload)) {
        return payload;
    }
    
    const uint8_t* ptr = data;
    
    std::memcpy(payload.ip_address, ptr, sizeof(payload.ip_address));
    ptr += sizeof(payload.ip_address);
    std::memcpy(payload.netmask, ptr, sizeof(payload.netmask));
    ptr += sizeof(payload.netmask);
    std::memcpy(payload.gateway, ptr, sizeof(payload.gateway));
    ptr += sizeof(payload.gateway);
    std::memcpy(&payload.mode, ptr, sizeof(payload.mode));
    ptr += sizeof(payload.mode);
    std::memcpy(payload.reserved, ptr, sizeof(payload.reserved));
    
    return payload;
}

// SetStaticIPPayload implementation
SetStaticIPPayload::SetStaticIPPayload() {
    std::memset(ip_address, 0, sizeof(ip_address));
    std::memset(netmask, 0, sizeof(netmask));
    std::memset(gateway, 0, sizeof(gateway));
}

std::vector<uint8_t> SetStaticIPPayload::serialize() const {
    std::vector<uint8_t> data(sizeof(SetStaticIPPayload));
    uint8_t* ptr = data.data();
    
    std::memcpy(ptr, ip_address, sizeof(ip_address));
    ptr += sizeof(ip_address);
    std::memcpy(ptr, netmask, sizeof(netmask));
    ptr += sizeof(netmask);
    std::memcpy(ptr, gateway, sizeof(gateway));
    
    return data;
}

SetStaticIPPayload SetStaticIPPayload::deserialize(const uint8_t* data, size_t len) {
    SetStaticIPPayload payload;
    if (len < sizeof(SetStaticIPPayload)) {
        return payload;
    }
    
    const uint8_t* ptr = data;
    
    std::memcpy(payload.ip_address, ptr, sizeof(payload.ip_address));
    ptr += sizeof(payload.ip_address);
    std::memcpy(payload.netmask, ptr, sizeof(payload.netmask));
    ptr += sizeof(payload.netmask);
    std::memcpy(payload.gateway, ptr, sizeof(payload.gateway));
    
    return payload;
}

// SetDHCPServerPayload implementation
SetDHCPServerPayload::SetDHCPServerPayload() {
    std::memset(server_ip, 0, sizeof(server_ip));
    std::memset(range_start, 0, sizeof(range_start));
    std::memset(range_end, 0, sizeof(range_end));
    std::memset(netmask, 0, sizeof(netmask));
    std::memset(gateway, 0, sizeof(gateway));
}

std::vector<uint8_t> SetDHCPServerPayload::serialize() const {
    std::vector<uint8_t> data(sizeof(SetDHCPServerPayload));
    uint8_t* ptr = data.data();
    
    std::memcpy(ptr, server_ip, sizeof(server_ip));
    ptr += sizeof(server_ip);
    std::memcpy(ptr, range_start, sizeof(range_start));
    ptr += sizeof(range_start);
    std::memcpy(ptr, range_end, sizeof(range_end));
    ptr += sizeof(range_end);
    std::memcpy(ptr, netmask, sizeof(netmask));
    ptr += sizeof(netmask);
    std::memcpy(ptr, gateway, sizeof(gateway));
    
    return data;
}

SetDHCPServerPayload SetDHCPServerPayload::deserialize(const uint8_t* data, size_t len) {
    SetDHCPServerPayload payload;
    if (len < sizeof(SetDHCPServerPayload)) {
        return payload;
    }
    
    const uint8_t* ptr = data;
    
    std::memcpy(payload.server_ip, ptr, sizeof(payload.server_ip));
    ptr += sizeof(payload.server_ip);
    std::memcpy(payload.range_start, ptr, sizeof(payload.range_start));
    ptr += sizeof(payload.range_start);
    std::memcpy(payload.range_end, ptr, sizeof(payload.range_end));
    ptr += sizeof(payload.range_end);
    std::memcpy(payload.netmask, ptr, sizeof(payload.netmask));
    ptr += sizeof(payload.netmask);
    std::memcpy(payload.gateway, ptr, sizeof(payload.gateway));
    
    return payload;
}

// ConfigResponsePayload implementation
ConfigResponsePayload::ConfigResponsePayload() 
    : status(StatusCode::SUCCESS) {
    std::memset(message, 0, sizeof(message));
    std::memset(reserved, 0, sizeof(reserved));
}

ConfigResponsePayload::ConfigResponsePayload(StatusCode code, const std::string& msg)
    : status(code) {
    std::memset(message, 0, sizeof(message));
    std::strncpy(message, msg.c_str(), sizeof(message) - 1);
    std::memset(reserved, 0, sizeof(reserved));
}

std::vector<uint8_t> ConfigResponsePayload::serialize() const {
    std::vector<uint8_t> data(sizeof(ConfigResponsePayload));
    uint8_t* ptr = data.data();
    
    std::memcpy(ptr, &status, sizeof(status));
    ptr += sizeof(status);
    std::memcpy(ptr, message, sizeof(message));
    ptr += sizeof(message);
    std::memcpy(ptr, reserved, sizeof(reserved));
    
    return data;
}

ConfigResponsePayload ConfigResponsePayload::deserialize(const uint8_t* data, size_t len) {
    ConfigResponsePayload payload;
    if (len < sizeof(ConfigResponsePayload)) {
        return payload;
    }
    
    const uint8_t* ptr = data;
    
    std::memcpy(&payload.status, ptr, sizeof(payload.status));
    ptr += sizeof(payload.status);
    std::memcpy(payload.message, ptr, sizeof(payload.message));
    ptr += sizeof(payload.message);
    std::memcpy(payload.reserved, ptr, sizeof(payload.reserved));
    
    return payload;
}

// ErrorResponsePayload implementation
ErrorResponsePayload::ErrorResponsePayload() 
    : error_code(StatusCode::ERROR_UNKNOWN) {
    std::memset(error_message, 0, sizeof(error_message));
    std::memset(reserved, 0, sizeof(reserved));
}

ErrorResponsePayload::ErrorResponsePayload(StatusCode code, const std::string& msg)
    : error_code(code) {
    std::memset(error_message, 0, sizeof(error_message));
    std::strncpy(error_message, msg.c_str(), sizeof(error_message) - 1);
    std::memset(reserved, 0, sizeof(reserved));
}

std::vector<uint8_t> ErrorResponsePayload::serialize() const {
    std::vector<uint8_t> data(sizeof(ErrorResponsePayload));
    uint8_t* ptr = data.data();
    
    std::memcpy(ptr, &error_code, sizeof(error_code));
    ptr += sizeof(error_code);
    std::memcpy(ptr, error_message, sizeof(error_message));
    ptr += sizeof(error_message);
    std::memcpy(ptr, reserved, sizeof(reserved));
    
    return data;
}

ErrorResponsePayload ErrorResponsePayload::deserialize(const uint8_t* data, size_t len) {
    ErrorResponsePayload payload;
    if (len < sizeof(ErrorResponsePayload)) {
        return payload;
    }
    
    const uint8_t* ptr = data;
    
    std::memcpy(&payload.error_code, ptr, sizeof(payload.error_code));
    ptr += sizeof(payload.error_code);
    std::memcpy(payload.error_message, ptr, sizeof(payload.error_message));
    ptr += sizeof(payload.error_message);
    std::memcpy(payload.reserved, ptr, sizeof(payload.reserved));
    
    return payload;
}

} // namespace network_discovery
