#pragma once

#include <string>
#include <cstdint>
#include <vector>

// Handle struct packing for different compilers
#ifdef _MSC_VER
    #define PACK_START __pragma(pack(push, 1))
    #define PACK_END __pragma(pack(pop))
    #define PACKED
#else
    #define PACK_START
    #define PACK_END
    #define PACKED __attribute__((packed))
#endif

namespace network_discovery {

// Protocol constants
constexpr uint16_t DEFAULT_DISCOVERY_PORT = 47809;
constexpr uint32_t PROTOCOL_MAGIC = 0x4E445343; // "NDSC"
constexpr uint16_t PROTOCOL_VERSION = 1;

// Message types
enum class MessageType : uint8_t {
    DISCOVERY_REQUEST = 0x01,
    DISCOVERY_RESPONSE = 0x02,
    GET_IP_REQUEST = 0x03,
    GET_IP_RESPONSE = 0x04,
    SET_DHCP_REQUEST = 0x05,
    SET_STATIC_IP_REQUEST = 0x06,
    CONFIG_RESPONSE = 0x07,
    SET_DHCP_SERVER_REQUEST = 0x08,
    ERROR_RESPONSE = 0xFF
};

// Configuration modes
enum class NetworkMode : uint8_t {
    DHCP = 0x01,
    STATIC = 0x02,
    DHCP_SERVER = 0x03
};

// Status codes
enum class StatusCode : uint8_t {
    SUCCESS = 0x00,
    ERROR_INVALID_REQUEST = 0x01,
    ERROR_PERMISSION_DENIED = 0x02,
    ERROR_NETWORK_ERROR = 0x03,
    ERROR_UNKNOWN = 0xFF
};

// Message header (common to all messages)
PACK_START
struct MessageHeader {
    uint32_t magic;           // Protocol magic number
    uint16_t version;         // Protocol version
    MessageType type;         // Message type
    uint8_t reserved;         // Reserved for future use
    uint32_t payload_length;  // Length of payload following header
    
    MessageHeader();
    MessageHeader(MessageType msg_type, uint32_t payload_len);
    
    bool is_valid() const;
    std::vector<uint8_t> serialize() const;
    static MessageHeader deserialize(const uint8_t* data, size_t len);
} PACKED;
PACK_END

// Discovery response payload
PACK_START
struct DiscoveryResponsePayload {
    char server_name[64];     // Server hostname
    char ip_address[16];      // Server IP address (IPv4 string format)
    uint16_t service_port;    // Port for further communication
    NetworkMode current_mode; // Current network configuration mode
    uint8_t reserved;         // Reserved for alignment
    char serial_number[32];   // Server serial number
    
    DiscoveryResponsePayload();
    std::vector<uint8_t> serialize() const;
    static DiscoveryResponsePayload deserialize(const uint8_t* data, size_t len);
} PACKED;
PACK_END

// Get IP response payload
PACK_START
struct GetIPResponsePayload {
    char ip_address[16];      // Current IP address
    char netmask[16];         // Network mask
    char gateway[16];         // Default gateway
    NetworkMode mode;         // Current mode (DHCP/Static)
    uint8_t reserved[3];      // Reserved for alignment
    
    GetIPResponsePayload();
    std::vector<uint8_t> serialize() const;
    static GetIPResponsePayload deserialize(const uint8_t* data, size_t len);
} PACKED;
PACK_END

// Set static IP request payload
PACK_START
struct SetStaticIPPayload {
    char ip_address[16];      // IP address to set
    char netmask[16];         // Network mask
    char gateway[16];         // Default gateway
    
    SetStaticIPPayload();
    std::vector<uint8_t> serialize() const;
    static SetStaticIPPayload deserialize(const uint8_t* data, size_t len);
} PACKED;
PACK_END

// Set DHCP server request payload
PACK_START
struct SetDHCPServerPayload {
    char server_ip[16];       // DHCP server IP address
    char range_start[16];     // DHCP range start
    char range_end[16];       // DHCP range end
    char netmask[16];         // Network mask
    char gateway[16];         // Gateway (usually server IP)
    
    SetDHCPServerPayload();
    std::vector<uint8_t> serialize() const;
    static SetDHCPServerPayload deserialize(const uint8_t* data, size_t len);
} PACKED;
PACK_END

// Configuration response payload
PACK_START
struct ConfigResponsePayload {
    StatusCode status;        // Operation status
    char message[128];        // Status message
    uint8_t reserved[3];      // Reserved for alignment
    
    ConfigResponsePayload();
    ConfigResponsePayload(StatusCode code, const std::string& msg);
    std::vector<uint8_t> serialize() const;
    static ConfigResponsePayload deserialize(const uint8_t* data, size_t len);
} PACKED;
PACK_END

// Error response payload
PACK_START
struct ErrorResponsePayload {
    StatusCode error_code;    // Error code
    char error_message[256];  // Error description
    uint8_t reserved[3];      // Reserved for alignment
    
    ErrorResponsePayload();
    ErrorResponsePayload(StatusCode code, const std::string& msg);
    std::vector<uint8_t> serialize() const;
    static ErrorResponsePayload deserialize(const uint8_t* data, size_t len);
} PACKED;
PACK_END

} // namespace network_discovery
