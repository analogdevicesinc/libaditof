#include "platform.h"
#include <iostream>

#ifdef PLATFORM_WINDOWS
#include <sstream>
#else
#include <cstring>
#include <errno.h>
#endif

namespace network_discovery {

bool Platform::initialize_networking() {
#ifdef PLATFORM_WINDOWS
    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (result != 0) {
        std::cerr << "WSAStartup failed: " << result << std::endl;
        return false;
    }
    return true;
#else
    // No initialization needed on Linux
    return true;
#endif
}

void Platform::cleanup_networking() {
#ifdef PLATFORM_WINDOWS
    WSACleanup();
#else
    // No cleanup needed on Linux
#endif
}

int Platform::get_last_error() {
#ifdef PLATFORM_WINDOWS
    return WSAGetLastError();
#else
    return errno;
#endif
}

std::string Platform::get_error_string(int error_code) {
#ifdef PLATFORM_WINDOWS
    char* message = nullptr;
    FormatMessageA(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        nullptr,
        error_code,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPSTR)&message,
        0,
        nullptr
    );
    std::string result = message ? message : "Unknown error";
    LocalFree(message);
    return result;
#else
    return std::strerror(error_code);
#endif
}

bool Platform::has_admin_privileges() {
#ifdef PLATFORM_WINDOWS
    BOOL isAdmin = FALSE;
    PSID adminGroup = nullptr;
    SID_IDENTIFIER_AUTHORITY ntAuthority = SECURITY_NT_AUTHORITY;
    
    if (AllocateAndInitializeSid(&ntAuthority, 2, SECURITY_BUILTIN_DOMAIN_RID,
                                  DOMAIN_ALIAS_RID_ADMINS, 0, 0, 0, 0, 0, 0, &adminGroup)) {
        CheckTokenMembership(nullptr, adminGroup, &isAdmin);
        FreeSid(adminGroup);
    }
    
    return isAdmin == TRUE;
#else
    return geteuid() == 0;
#endif
}

} // namespace network_discovery
