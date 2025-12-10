#include "network_utils.h"
#include "platform.h"
#include <cstring>
#include <iostream>

#ifdef PLATFORM_LINUX
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <net/if.h>
#include <ifaddrs.h>
#include <unistd.h>
#include <fstream>
#include <sstream>
#endif

namespace network_discovery {

std::vector<std::string> NetworkUtils::get_local_ip_addresses() {
    std::vector<std::string> addresses;
    
#ifdef PLATFORM_WINDOWS
    char hostname[256];
    if (gethostname(hostname, sizeof(hostname)) == 0) {
        struct addrinfo hints, *result;
        ZeroMemory(&hints, sizeof(hints));
        hints.ai_family = AF_INET;
        hints.ai_socktype = SOCK_DGRAM;
        
        if (getaddrinfo(hostname, nullptr, &hints, &result) == 0) {
            for (struct addrinfo* ptr = result; ptr != nullptr; ptr = ptr->ai_next) {
                struct sockaddr_in* addr = (struct sockaddr_in*)ptr->ai_addr;
                char ip_str[INET_ADDRSTRLEN];
                inet_ntop(AF_INET, &addr->sin_addr, ip_str, INET_ADDRSTRLEN);
                std::string ip(ip_str);
                if (ip != "127.0.0.1") {
                    addresses.push_back(ip);
                }
            }
            freeaddrinfo(result);
        }
    }
#else
    struct ifaddrs* ifaddr;
    
    if (getifaddrs(&ifaddr) == -1) {
        return addresses;
    }
    
    for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;
        
        // Only IPv4
        if (ifa->ifa_addr->sa_family == AF_INET) {
            char addr_str[INET_ADDRSTRLEN];
            void* addr_ptr = &((struct sockaddr_in*)ifa->ifa_addr)->sin_addr;
            inet_ntop(AF_INET, addr_ptr, addr_str, INET_ADDRSTRLEN);
            
            std::string addr(addr_str);
            // Skip loopback
            if (addr != "127.0.0.1") {
                addresses.push_back(addr);
            }
        }
    }
    
    freeifaddrs(ifaddr);
#endif
    return addresses;
}

std::string NetworkUtils::get_primary_ip_address() {
    auto addresses = get_local_ip_addresses();
    return addresses.empty() ? "127.0.0.1" : addresses[0];
}

std::string NetworkUtils::get_hostname() {
    char hostname[256];
    if (gethostname(hostname, sizeof(hostname)) == 0) {
        return std::string(hostname);
    }
    return "unknown";
}

std::string NetworkUtils::get_default_interface() {
#ifdef PLATFORM_WINDOWS
    // On Windows, we'll use the first non-loopback adapter
    PIP_ADAPTER_INFO adapterInfo = nullptr;
    ULONG bufLen = sizeof(IP_ADAPTER_INFO);
    adapterInfo = (IP_ADAPTER_INFO*)malloc(bufLen);
    
    if (GetAdaptersInfo(adapterInfo, &bufLen) == ERROR_BUFFER_OVERFLOW) {
        free(adapterInfo);
        adapterInfo = (IP_ADAPTER_INFO*)malloc(bufLen);
    }
    
    std::string result = "eth0"; // Fallback
    if (GetAdaptersInfo(adapterInfo, &bufLen) == NO_ERROR) {
        PIP_ADAPTER_INFO adapter = adapterInfo;
        while (adapter) {
            if (adapter->Type == IF_TYPE_ETHERNET_CSMACD || adapter->Type == IF_TYPE_IEEE80211) {
                result = adapter->AdapterName;
                break;
            }
            adapter = adapter->Next;
        }
    }
    
    free(adapterInfo);
    return result;
#else
    std::ifstream route_file("/proc/net/route");
    std::string line;
    
    // Skip header
    std::getline(route_file, line);
    
    while (std::getline(route_file, line)) {
        std::istringstream iss(line);
        std::string iface, dest;
        iss >> iface >> dest;
        
        // Default route has destination 00000000
        if (dest == "00000000") {
            return iface;
        }
    }
    
    return "eth0"; // Fallback
#endif
}

NetworkUtils::NetworkInfo NetworkUtils::get_network_info(const std::string& interface_name) {
    NetworkInfo info;
    std::string iface = interface_name.empty() ? get_default_interface() : interface_name;
    info.interface_name = iface;
    
#ifdef PLATFORM_WINDOWS
    PIP_ADAPTER_INFO adapterInfo = nullptr;
    ULONG bufLen = sizeof(IP_ADAPTER_INFO);
    adapterInfo = (IP_ADAPTER_INFO*)malloc(bufLen);
    
    if (GetAdaptersInfo(adapterInfo, &bufLen) == ERROR_BUFFER_OVERFLOW) {
        free(adapterInfo);
        adapterInfo = (IP_ADAPTER_INFO*)malloc(bufLen);
    }
    
    if (GetAdaptersInfo(adapterInfo, &bufLen) == NO_ERROR) {
        PIP_ADAPTER_INFO adapter = adapterInfo;
        while (adapter) {
            if (iface.empty() || std::string(adapter->AdapterName) == iface) {
                info.ip_address = adapter->IpAddressList.IpAddress.String;
                info.netmask = adapter->IpAddressList.IpMask.String;
                info.gateway = adapter->GatewayList.IpAddress.String;
                info.is_dhcp = (adapter->DhcpEnabled != 0);
                break;
            }
            adapter = adapter->Next;
        }
    }
    
    free(adapterInfo);
#else
    struct ifaddrs* ifaddr;
    if (getifaddrs(&ifaddr) == -1) {
        return info;
    }
    
    for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;
        if (std::string(ifa->ifa_name) != iface) continue;
        
        if (ifa->ifa_addr->sa_family == AF_INET) {
            // IP address
            char addr_str[INET_ADDRSTRLEN];
            void* addr_ptr = &((struct sockaddr_in*)ifa->ifa_addr)->sin_addr;
            inet_ntop(AF_INET, addr_ptr, addr_str, INET_ADDRSTRLEN);
            info.ip_address = addr_str;
            
            // Netmask
            if (ifa->ifa_netmask) {
                void* mask_ptr = &((struct sockaddr_in*)ifa->ifa_netmask)->sin_addr;
                inet_ntop(AF_INET, mask_ptr, addr_str, INET_ADDRSTRLEN);
                info.netmask = addr_str;
            }
        }
    }
    
    freeifaddrs(ifaddr);
    
    // Try to get gateway
    std::ifstream route_file("/proc/net/route");
    std::string line;
    std::getline(route_file, line); // Skip header
    
    while (std::getline(route_file, line)) {
        std::istringstream iss(line);
        std::string route_iface, dest, gateway_hex;
        iss >> route_iface >> dest >> gateway_hex;
        
        if (route_iface == iface && dest == "00000000") {
            // Parse hex gateway address
            unsigned long gw = std::stoul(gateway_hex, nullptr, 16);
            struct in_addr gw_addr;
            gw_addr.s_addr = gw;
            info.gateway = inet_ntoa(gw_addr);
            break;
        }
    }
    
    // Check if DHCP is enabled (simple heuristic)
    std::string dhcp_lease_path = "/var/lib/dhcp/dhclient." + iface + ".leases";
    std::ifstream dhcp_file(dhcp_lease_path);
    info.is_dhcp = dhcp_file.good();
#endif
    
    return info;
}

bool NetworkUtils::has_root_privileges() {
    return Platform::has_admin_privileges();
}

bool NetworkUtils::set_static_ip(const std::string& interface_name,
                                 const std::string& ip_address,
                                 const std::string& netmask,
                                 const std::string& gateway) {
    if (!has_root_privileges()) {
        std::cerr << "Administrator privileges required to set static IP" << std::endl;
        return false;
    }
    
#ifdef PLATFORM_WINDOWS
    // Windows: Use netsh command
    std::string cmd = "netsh interface ip set address \"" + interface_name + 
                     "\" static " + ip_address + " " + netmask + " " + gateway;
    return system(cmd.c_str()) == 0;
#else
    // Bring interface down
    std::string cmd = "ip link set " + interface_name + " down 2>&1";
    int ret = system(cmd.c_str());
    (void)ret; // Intentionally ignored
    
    // Set IP address
    cmd = "ip addr flush dev " + interface_name + " 2>&1";
    ret = system(cmd.c_str());
    (void)ret; // Intentionally ignored
    
    cmd = "ip addr add " + ip_address + "/" + netmask + " dev " + interface_name + " 2>&1";
    if (system(cmd.c_str()) != 0) {
        return false;
    }
    
    // Bring interface up
    cmd = "ip link set " + interface_name + " up 2>&1";
    if (system(cmd.c_str()) != 0) {
        return false;
    }
    
    // Set default gateway
    cmd = "ip route add default via " + gateway + " 2>&1";
    ret = system(cmd.c_str()); // May fail if route already exists
    (void)ret; // Intentionally ignored
    
    return true;
#endif
}

bool NetworkUtils::enable_dhcp(const std::string& interface_name) {
    if (!has_root_privileges()) {
        std::cerr << "Administrator privileges required to enable DHCP" << std::endl;
        return false;
    }
    
#ifdef PLATFORM_WINDOWS
    // Windows: Use netsh command
    std::string cmd = "netsh interface ip set address \"" + interface_name + "\" dhcp";
    return system(cmd.c_str()) == 0;
#else
    // Kill existing DHCP client
    std::string cmd = "pkill -f 'dhclient.*" + interface_name + "' 2>&1";
    int ret = system(cmd.c_str());
    (void)ret; // Intentionally ignored
    
    // Flush existing IP
    cmd = "ip addr flush dev " + interface_name + " 2>&1";
    ret = system(cmd.c_str());
    (void)ret; // Intentionally ignored
    
    // Start DHCP client
    cmd = "dhclient " + interface_name + " 2>&1";
    return system(cmd.c_str()) == 0;
#endif
}

bool NetworkUtils::start_dhcp_server(const std::string& interface_name,
                                     const std::string& server_ip,
                                     const std::string& range_start,
                                     const std::string& range_end,
                                     const std::string& netmask,
                                     const std::string& gateway) {
    if (!has_root_privileges()) {
        std::cerr << "Administrator privileges required to start DHCP server" << std::endl;
        return false;
    }
    
#ifdef PLATFORM_WINDOWS
    std::cerr << "DHCP server mode not supported on Windows" << std::endl;
    return false;
#else
    // First, set static IP on the interface
    if (!set_static_ip(interface_name, server_ip, netmask, gateway)) {
        std::cerr << "Failed to set static IP for DHCP server" << std::endl;
        return false;
    }
    
    // Stop any existing DHCP server
    stop_dhcp_server(interface_name);
    
    // Create dnsmasq configuration file
    std::string config_file = "/tmp/dnsmasq_" + interface_name + ".conf";
    std::ofstream conf(config_file);
    if (!conf.is_open()) {
        std::cerr << "Failed to create dnsmasq config file" << std::endl;
        return false;
    }
    
    // Write dnsmasq configuration
    conf << "# Dnsmasq configuration for " << interface_name << std::endl;
    conf << "interface=" << interface_name << std::endl;
    conf << "bind-interfaces" << std::endl;
    conf << "dhcp-range=" << range_start << "," << range_end << ",12h" << std::endl;
    conf << "dhcp-option=option:router," << gateway << std::endl;
    conf << "dhcp-option=option:dns-server," << server_ip << std::endl;
    conf << "no-daemon" << std::endl;
    conf << "log-queries" << std::endl;
    conf << "log-dhcp" << std::endl;
    conf << "pid-file=/tmp/dnsmasq_" << interface_name << ".pid" << std::endl;
    conf.close();
    
    // Start dnsmasq in background
    std::string cmd = "dnsmasq --conf-file=" + config_file + " > /dev/null 2>&1 &";
    int ret = system(cmd.c_str());
    
    if (ret != 0) {
        std::cerr << "Failed to start dnsmasq DHCP server" << std::endl;
        return false;
    }
    
    // Give it a moment to start
    usleep(500000); // 500ms
    
    // Check if it's running
    return is_dhcp_server_running(interface_name);
#endif
}

bool NetworkUtils::stop_dhcp_server(const std::string& interface_name) {
    if (!has_root_privileges()) {
        std::cerr << "Administrator privileges required to stop DHCP server" << std::endl;
        return false;
    }
    
#ifdef PLATFORM_WINDOWS
    return true; // Not supported on Windows
#else
    // Read PID file
    std::string pid_file = "/tmp/dnsmasq_" + interface_name + ".pid";
    std::ifstream pf(pid_file);
    if (pf.is_open()) {
        std::string pid;
        std::getline(pf, pid);
        pf.close();
        
        if (!pid.empty()) {
            std::string cmd = "kill " + pid + " 2>&1";
            if (system(cmd.c_str()) != 0) {
                return false;
            }
        }
        
        // Remove PID file
        std::remove(pid_file.c_str());
    }
    
    // Also try pkill as fallback
    std::string cmd = "pkill -f 'dnsmasq.*" + interface_name + "' 2>&1";
    if (system(cmd.c_str()) != 0) {
        return false;
    }
    
    // Remove config file
    std::string config_file = "/tmp/dnsmasq_" + interface_name + ".conf";
    std::remove(config_file.c_str());
    
    return true;
#endif
}

bool NetworkUtils::is_dhcp_server_running(const std::string& interface_name) {
#ifdef PLATFORM_WINDOWS
    return false;
#else
    std::string pid_file = "/tmp/dnsmasq_" + interface_name + ".pid";
    std::ifstream pf(pid_file);
    if (!pf.is_open()) {
        return false;
    }
    
    std::string pid;
    std::getline(pf, pid);
    pf.close();
    
    if (pid.empty()) {
        return false;
    }
    
    // Check if process is running
    std::string cmd = "kill -0 " + pid + " 2>/dev/null";
    return system(cmd.c_str()) == 0;
#endif
}

int NetworkUtils::create_broadcast_socket(uint16_t port, bool bind_socket) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        return -1;
    }
    
    // Enable broadcast
    int broadcast_enable = 1;
#ifdef PLATFORM_WINDOWS
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (const char*)&broadcast_enable, 
                   sizeof(broadcast_enable)) < 0) {
#else
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, 
                   sizeof(broadcast_enable)) < 0) {
#endif
        close(sock);
        return -1;
    }
    
    // Set reuse address
    int reuse = 1;
#ifdef PLATFORM_WINDOWS
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) < 0) {
#else
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
#endif
        close(sock);
        return -1;
    }
    
    if (bind_socket) {
        struct sockaddr_in addr;
        std::memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = INADDR_ANY;
        addr.sin_port = htons(port);
        
        if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            close(sock);
            return -1;
        }
    }
    
    return sock;
}

bool NetworkUtils::bind_socket_to_interface(int socket_fd, const std::string& interface) {
    if (interface.empty()) {
        return true; // No interface specified, use default behavior
    }
    
#ifdef PLATFORM_LINUX
    // Bind socket to specific interface using SO_BINDTODEVICE
    if (setsockopt(socket_fd, SOL_SOCKET, SO_BINDTODEVICE, 
                   interface.c_str(), interface.length()) < 0) {
        std::cerr << "Warning: Failed to bind to interface " << interface 
                  << ": " << strerror(errno) << std::endl;
        std::cerr << "Note: Binding to specific interface requires root privileges" << std::endl;
        return false;
    }
    std::cout << "Socket bound to interface: " << interface << std::endl;
    return true;
#else
    // Windows doesn't support SO_BINDTODEVICE in the same way
    // Would need to enumerate interfaces and bind to specific IP
    std::cout << "Note: Interface binding not supported on Windows, using default" << std::endl;
    return true;
#endif
}

bool NetworkUtils::send_broadcast(int socket_fd, uint16_t port, 
                                  const void* data, size_t len) {
    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_BROADCAST;
    addr.sin_port = htons(port);
    
#ifdef PLATFORM_WINDOWS
    int sent = sendto(socket_fd, (const char*)data, (int)len, 0,
                      (struct sockaddr*)&addr, sizeof(addr));
    return sent == static_cast<int>(len);
#else
    ssize_t sent = sendto(socket_fd, data, len, 0,
                          (struct sockaddr*)&addr, sizeof(addr));
    return sent == static_cast<ssize_t>(len);
#endif
}

std::string NetworkUtils::get_broadcast_address(const std::string& interface_name) {
    std::string iface = interface_name.empty() ? get_default_interface() : interface_name;
    
#ifdef PLATFORM_WINDOWS
    // On Windows, just return generic broadcast
    return "255.255.255.255";
#else
    struct ifaddrs* ifaddr;
    if (getifaddrs(&ifaddr) == -1) {
        return "255.255.255.255";
    }
    
    std::string broadcast_addr = "255.255.255.255";
    
    for (struct ifaddrs* ifa = ifaddr; ifa != nullptr; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr == nullptr) continue;
        if (std::string(ifa->ifa_name) != iface) continue;
        
        if (ifa->ifa_addr->sa_family == AF_INET && ifa->ifa_broadaddr) {
            char addr_str[INET_ADDRSTRLEN];
            void* addr_ptr = &((struct sockaddr_in*)ifa->ifa_broadaddr)->sin_addr;
            inet_ntop(AF_INET, addr_ptr, addr_str, INET_ADDRSTRLEN);
            broadcast_addr = addr_str;
            break;
        }
    }
    
    freeifaddrs(ifaddr);
    return broadcast_addr;
#endif
}

} // namespace network_discovery
