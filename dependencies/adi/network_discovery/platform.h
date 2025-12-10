#pragma once

// Platform detection
#ifdef _WIN32
    #define PLATFORM_WINDOWS
#else
    #define PLATFORM_LINUX
#endif

// Windows-specific includes and definitions
#ifdef PLATFORM_WINDOWS
    #define WIN32_LEAN_AND_MEAN
    #define _WINSOCK_DEPRECATED_NO_WARNINGS
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <iphlpapi.h>
    #include <windows.h>
    
    // Link with required libraries
    #pragma comment(lib, "ws2_32.lib")
    #pragma comment(lib, "iphlpapi.lib")
    
    // Windows doesn't have these, define them
    #define sleep(x) Sleep((x) * 1000)
    #define close(x) closesocket(x)
    
    // POSIX-style types
    typedef int socklen_t;
    
#else
    // Linux includes
    #include <sys/socket.h>
    #include <sys/ioctl.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <net/if.h>
    #include <ifaddrs.h>
    #include <unistd.h>
#endif

#include <string>

namespace network_discovery {

class Platform {
public:
    // Initialize platform-specific networking (call once at startup)
    static bool initialize_networking();
    
    // Cleanup platform-specific networking (call at shutdown)
    static void cleanup_networking();
    
    // Get last socket error
    static int get_last_error();
    
    // Get error string
    static std::string get_error_string(int error_code);
    
    // Check if running with admin/root privileges
    static bool has_admin_privileges();
};

} // namespace network_discovery
