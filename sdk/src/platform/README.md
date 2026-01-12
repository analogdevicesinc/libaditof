# Platform (SoC) Layer

## Overview

The Platform layer provides SoC-specific device discovery and system information APIs for the ADCAM ToF SDK. This layer abstracts differences between embedded platforms (NVIDIA Jetson, NXP i.MX, Raspberry Pi, etc.) so the SDK can run on multiple hardware targets without modification.

## Responsibilities

| Layer | Responsibility | Examples |
|-------|---------------|----------|
| **Platform** (this layer) | SoC-specific device enumeration & system info | V4L2 device discovery, media-ctl parsing, boot version reading |
| **Connections/Target** | Hardware driver wrappers | V4L2 ioctls, ADSD3500 register access, GPIO control |
| **Cameras** | High-level camera logic | Frame acquisition, mode switching, calibration |

---

## Architecture

```
libaditof/sdk/src/platform/
├── README.md                          # This file
├── platform_interface.h               # Abstract platform interface
├── platform_factory.h                 # Factory for runtime platform selection
├── platform_factory.cpp
├── sensor_enumerator.h                # Base enumerator (platform-agnostic)
├── sensor_enumerator.cpp              # Implements SensorEnumeratorInterface
│
├── nvidia/                            # NVIDIA Jetson Orin Nano
│   ├── nvidia_platform.h              # Platform implementation header
│   ├── nvidia_platform.cpp            # NVIDIA-specific helpers
│   └── sensor_enumerator_nvidia.cpp   # searchSensors() implementation
│
├── imx/                               # NXP i.MX8MP
│   ├── imx_platform.h
│   ├── imx_platform.cpp
│   └── sensor_enumerator_imx.cpp
│
└── host/                              # Host/offline simulation
    ├── host_platform.h
    └── host_platform.cpp
```

---

## Platform Interface

### Core Abstraction

```cpp
// platform/platform_interface.h

namespace aditof {
namespace platform {

struct PlatformInfo {
    std::string name;              // "NVIDIA Jetson Orin Nano"
    std::string architecture;      // "aarch64"
    std::string videoDevicePrefix; // "vi-output" (NVIDIA), "mxc_isi" (NXP)
    std::string mediaController;   // "/dev/media*"
    std::vector<std::string> capabilities;
};

class IPlatform {
public:
    virtual ~IPlatform() = default;
    
    // Platform identification
    virtual PlatformInfo getPlatformInfo() const = 0;
    virtual std::string getBootloaderVersion() const = 0;
    virtual std::string getKernelVersion() const = 0;
    virtual std::string getSDCardVersion() const = 0;
    
    // Device discovery (SoC-specific)
    virtual Status findToFSensors(std::vector<SensorInfo>& sensors) = 0;
    virtual Status findRGBSensors(std::vector<RGBSensorInfo>& sensors) = 0;
    
    // Media pipeline parsing
    virtual Status parseMediaPipeline(
        const std::string& mediaDevice,
        std::string& devPath,
        std::string& subdevPath,
        std::string& deviceName) = 0;
};

}} // namespace aditof::platform
```

### Factory Pattern

```cpp
// Usage in SDK
#include "platform/platform_factory.h"

auto platform = aditof::platform::PlatformFactory::create();
auto info = platform->getPlatformInfo();
std::cout << "Running on: " << info.name << std::endl;

std::vector<SensorInfo> sensors;
platform->findToFSensors(sensors);
```

---

## Adding New Platform Support

See the "Adding New Platform Support" section in the original documentation for step-by-step instructions on adding support for new SoC platforms.

---

## Platform Detection

### Compile-Time Selection

Controlled by CMake flags:
- `-DNVIDIA=ON` → NVIDIA Jetson platform
- `-DNXP=ON` → NXP i.MX platform
- `-DRASPBERRY_PI=ON` → Raspberry Pi platform
- No flags → Host simulation

---

## References

- NVIDIA Jetson Linux API: https://docs.nvidia.com/jetson/l4t-multimedia/
- NXP i.MX Camera Guide: https://www.nxp.com/docs/en/user-guide/IMX_LINUX_USERS_GUIDE.pdf
- V4L2 Media Controller: https://www.kernel.org/doc/html/latest/userspace-api/media/mediactl/
