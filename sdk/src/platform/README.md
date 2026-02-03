# Platform (SoC) Layer

## Overview

The Platform layer provides SoC-specific device discovery and system information APIs for the ADCAM ToF SDK. This layer abstracts differences between embedded platforms (NVIDIA Jetson, NXP i.MX, Raspberry Pi, etc.) so the SDK can run on multiple hardware targets without modification.

## Responsibilities

| Layer | Responsibility | Examples |
|-------|---------------|----------|
| **Platform** (this layer) | SoC-specific device enumeration & system info | V4L2 device discovery, media-ctl parsing, boot version reading, GPIO reset |
| **Connections/Target** | Hardware driver wrappers | V4L2 ioctls, ADSD3500 register access, buffer management |
| **Cameras** | High-level camera logic | Frame acquisition, mode switching, calibration |

---

## Architecture

### Current Implementation (Unified Single-Platform Build)

```
libaditof/sdk/src/platform/
├── README.md                          # This file
├── platform.cmake                     # CMake configuration (sets PLATFORM_* variables)
├── platform_config.h.in               # Template for platform_config.h generation
├── platform_impl.h                    # Unified platform implementation header
├── platform_impl.cpp                  # Platform implementation (configured via CMake)
├── sensor_enumerator.h                # Platform-based sensor enumerator header
├── sensor_enumerator.cpp              # Implements SensorEnumeratorInterface
│
└── raspberrypi/                       # Raspberry Pi specific helpers
    ├── rpi_media_config.h             # RP1 CFE media pipeline configuration
    └── rpi_media_config.cpp           # media-ctl wrapper for format setup
```

**Key Design**: Instead of separate platform subdirectories, the implementation uses **compile-time configuration** via CMake. The `platform.cmake` file sets platform-specific constants (device names, GPIO pins, etc.) which are substituted into `platform_config.h` at build time.

---

## Platform Interface

### Core Data Structures

```cpp
// platform/platform_impl.h

namespace aditof {
namespace platform {

struct PlatformInfo {
    std::string name;              // "NVIDIA Jetson Orin Nano", "Raspberry Pi 5"
    std::string architecture;      // "aarch64"
    std::string videoDevicePrefix; // "vi-output" (NVIDIA), "rp1-cfe" (RPI), "mxc_isi" (NXP)
    std::string mediaController;   // "/dev/media*"
};

enum class SensorType { 
    SENSOR_ADSD3500 
};

struct SensorInfo {
    SensorType sensorType;
    std::string driverPath;        // e.g., "/dev/video0"
    std::string subDevPath;        // e.g., "/dev/v4l-subdev2"
    std::string captureDev;        // e.g., "vi-output, adsd3500"
};

struct RGBSensorInfo {
    std::string devicePath;
    std::string sensorName;
};

}} // namespace aditof::platform
```

### Platform Class (Singleton)

```cpp
class Platform {
public:
    // System information
    PlatformInfo getPlatformInfo() const;
    std::string getBootloaderVersion() const;
    std::string getKernelVersion() const;
    std::string getSDCardVersion() const;
    
    // Device discovery
    Status findToFSensors(std::vector<SensorInfo>& sensors);
    Status findRGBSensors(std::vector<RGBSensorInfo>& sensors);
    
    // Hardware control
    Status resetSensor(bool waitForInterrupt = false, 
                      bool* resetDone = nullptr,
                      int timeoutSeconds = 10);
    
    // Singleton access
    static Platform& getInstance();
    
private:
    Status parseMediaPipeline(const std::string& mediaDevice,
                             std::string& devPath,
                             std::string& subdevPath,
                             std::string& deviceName);
    std::string getVersionOfComponent(const std::string& component) const;
};
```

### Sensor Enumerator (SDK Integration)

```cpp
// sensor_enumerator.h - Bridges Platform to SDK's SensorEnumeratorInterface

class PlatformSensorEnumerator : public SensorEnumeratorInterface {
public:
    Status searchSensors() override;
    Status getDepthSensors(std::vector<std::shared_ptr<DepthSensorInterface>>& depthSensors) override;
    Status getUbootVersion(std::string& uBootVersion) const override;
    Status getKernelVersion(std::string& kernelVersion) const override;
    Status getSdVersion(std::string& sdVersion) const override;
    
private:
    platform::Platform m_platform;
    std::vector<platform::SensorInfo> m_sensorsInfo;
};
```

### Usage Example

```cpp
// In SDK initialization code
#include "platform/platform_impl.h"

// Get platform information
auto& platform = aditof::platform::Platform::getInstance();
auto info = platform.getPlatformInfo();
LOG(INFO) << "Running on: " << info.name;

// Discover sensors
std::vector<aditof::platform::SensorInfo> sensors;
platform.findToFSensors(sensors);

for (const auto& sensor : sensors) {
    LOG(INFO) << "Found sensor: " << sensor.driverPath;
}

// Reset sensor hardware
platform.resetSensor(true);  // Wait for interrupt
```

---

## Platform Configuration via CMake

### Build-Time Selection

Platforms are selected via CMake flags at **build time**:

```bash
# NVIDIA Jetson Orin Nano
cmake -DNVIDIA=ON ..

# Raspberry Pi 5
cmake -DRPI=ON ..

# NXP i.MX8MP
cmake -DNXP=ON ..
```

### Configuration Variables (`platform.cmake`)

Each platform defines the following in `platform.cmake`:

| Variable | NVIDIA | Raspberry Pi 5 | NXP i.MX8 |
|----------|--------|----------------|-----------|
| `PLATFORM_NAME` | "NVIDIA Jetson Orin Nano" | "Raspberry Pi 5" | "NXP i.MX 8" |
| `PLATFORM_CAPTURE_DEVICE` | "vi-output, adsd3500" | "rp1-cfe" | "mxc_isi" |
| `PLATFORM_VIDEO_PREFIX` | "vi-output" | "rp1-cfe" | "mxc_isi" |
| `PLATFORM_MEDIA_CONTROLLER` | "/dev/media" | "/dev/media" | "/dev/media" |
| `PLATFORM_RESET_GPIO` | "PAC.00" (named) | "" (debugfs) | "gpio64" |
| `PLATFORM_RESET_GPIO_PIN` | 0 | 34 (physical pin) | 0 |
| `PLATFORM_RESET_PULSE_US` | 100000 (100ms) | 100000 (100ms) | 1000000 (1s) |
| `PLATFORM_RESET_DELAY_US` | 10000000 (10s) | 2000000 (2s) | 7000000 (7s) |

These variables are substituted into `platform_config.h.in` during CMake configuration, generating `build/libaditof/sdk/platform_config.h`.

---

## Device Discovery Implementation

### Media Pipeline Parsing

The platform layer uses `media-ctl -p` to discover the V4L2 media controller topology:

```cpp
Status Platform::findToFSensors(std::vector<SensorInfo>& sensors) {
    // 1. Scan /dev for media devices (/dev/media0, /dev/media1, ...)
    DIR* dir = opendir("/dev");
    std::vector<std::string> mediaDevices;
    while ((entry = readdir(dir)) != nullptr) {
        if (strncmp(entry->d_name, "media", 5) == 0) {
            mediaDevices.push_back("/dev/" + entry->d_name);
        }
    }
    
    // 2. Parse each media device's topology
    for (const auto& mediaDevice : mediaDevices) {
        std::string devPath, subdevPath, deviceName;
        parseMediaPipeline(mediaDevice, devPath, subdevPath, deviceName);
        
        // 3. Match against platform-specific capture device name
        if (deviceName.find(PLATFORM_CAPTURE_DEVICE) != std::string::npos) {
            SensorInfo info;
            info.sensorType = SensorType::SENSOR_ADSD3500;
            info.driverPath = devPath;      // e.g., "/dev/video0"
            info.subDevPath = subdevPath;   // e.g., "/dev/v4l-subdev2"
            info.captureDev = deviceName;
            sensors.push_back(info);
        }
    }
}
```

### Parsing Strategy

The `parseMediaPipeline()` function parses `media-ctl -p` output to extract:
1. **Capture device** entity (e.g., "vi-output, adsd3500 10-0038" for NVIDIA)
2. **Video device node** associated with capture entity (`/dev/video0`)
3. **Subdevice node** for ADSD3500 sensor (`/dev/v4l-subdev2`)

**Example media-ctl output (Raspberry Pi 5)**:
```
- entity 5: rp1-cfe-csi2_ch0 (1 pad, 1 link)
            type Node subtype V4L
            device node name /dev/video0
            
- entity 15: adsd3500 10-0038 (1 pad, 1 link)
            type V4L2 subdev subtype Sensor
            device node name /dev/v4l-subdev2
```

---

## Sensor Reset via GPIO

### Platform-Specific GPIO Handling

Each platform requires different GPIO access methods:

#### **NVIDIA Jetson**
Uses **named GPIO** via character device API:
```cpp
#ifdef NVIDIA
// Open /dev/gpiochip0 and request line "PAC.00"
Gpio gpio("PAC.00");
gpio.setOutputValue(0);  // Assert reset (active-high)
usleep(100000);
gpio.setOutputValue(1);  // Deassert reset
#endif
```

#### **Raspberry Pi**
Uses **debugfs dynamic lookup** (pin name resolved at runtime):
```cpp
#ifdef RPI
// Read /sys/kernel/debug/gpio to find gpio chip + offset for physical pin 34
// Then use character device API
std::string gpioName = lookupGpioFromPhysicalPin(34);
Gpio gpio(gpioName);  // e.g., "gpiochip0:34"
gpio.setOutputValue(0);
usleep(100000);
gpio.setOutputValue(1);
#endif
```

#### **NXP i.MX8**
Uses **named GPIO line**:
```cpp
#ifdef NXP
Gpio gpio("gpio64");
gpio.setOutputValue(0);
usleep(1000000);  // 1s reset pulse
gpio.setOutputValue(1);
#endif
```

### Interrupt-Synchronized Reset

The `resetSensor()` method can optionally wait for a hardware interrupt from ADSD3500:

```cpp
Status Platform::resetSensor(bool waitForInterrupt, bool* resetDone, int timeoutSeconds) {
    // 1. Assert GPIO reset
    Gpio gpio(PLATFORM_RESET_GPIO);
    gpio.setOutputValue(0);  // Pull low
    usleep(PLATFORM_RESET_PULSE_US);
    gpio.setOutputValue(1);  // Release
    
    // 2. If waitForInterrupt=true, block until interrupt fires
    if (waitForInterrupt && resetDone != nullptr) {
        auto start = std::chrono::steady_clock::now();
        while (!*resetDone) {
            if (elapsed > timeoutSeconds) {
                LOG(ERROR) << "Reset interrupt timeout";
                return Status::GENERIC_ERROR;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } else {
        usleep(PLATFORM_RESET_DELAY_US);  // Fixed delay fallback
    }
    
    return Status::OK;
}
```

---

## Raspberry Pi Specific: Media Pipeline Configuration

Raspberry Pi 5's RP1 CFE (Camera Front End) requires explicit media-ctl format configuration before streaming to avoid `VIDIOC_STREAMON` format mismatch errors.

### RP1 Media Configuration Helper

```cpp
// raspberrypi/rpi_media_config.h

namespace aditof::platform::rpi {

bool configureMediaPipeline(
    const std::string& mediaDevice,     // "/dev/media0"
    const std::string& videoDevice,     // "/dev/video0"
    const std::string& sensorEntity,    // "adsd3500 10-0038"
    int width,                          // 1024
    int height,                         // 4096 (full V4L2 payload height)
    int bitDepth);                      // 12 or 8

std::string detectRpiMediaDevice();
std::string detectSensorEntity(const std::string& mediaDevice);

} // namespace aditof::platform::rpi
```

### Usage in Camera Initialization

```cpp
#ifdef RPI
// Before starting capture, configure RP1 CFE media pipeline
bool configured = rpi::configureMediaPipeline(
    "/dev/media0",
    "/dev/video0",
    "adsd3500 10-0038",
    1024,   // baseResolutionWidth
    4096,   // frameHeightInBytes (includes all interleaved planes)
    12      // bit depth
);

if (!configured) {
    LOG(ERROR) << "Failed to configure RP1 CFE media pipeline";
    return Status::GENERIC_ERROR;
}
#endif
```

This runs `media-ctl` commands to set consistent formats across:
- Sensor output pad (`adsd3500 10-0038:0`)
- CSI-2 receiver input/output (`rp1-csi2:0`, `rp1-csi2:4`)
- CFE output (`rp1-cfe-csi2_ch0:0`)

---

## Version Information Retrieval

### Bootloader Version
```cpp
std::string Platform::getBootloaderVersion() const {
    return getVersionOfComponent("u-boot");
}
```
Reads from `/etc/adi_<component>_version.txt` (platform-specific version files).

### Kernel Version
```cpp
std::string Platform::getKernelVersion() const {
    std::ifstream versionFile("/proc/version");
    std::string version;
    std::getline(versionFile, version);
    return version;
}
```
Parses `/proc/version` directly.

### SD Card Image Version
```cpp
std::string Platform::getSDCardVersion() const {
    return getVersionOfComponent("sd_img_ver");
}
```
Reads from `/etc/adi_sd_img_ver_version.txt`.

---

## Adding New Platform Support

### Step 1: Add CMake Configuration

Edit `platform.cmake`:

```cmake
elseif(YOUR_PLATFORM)
    set(PLATFORM_NAME "Your Platform Name")
    set(PLATFORM_CAPTURE_DEVICE "your-capture-device")
    set(PLATFORM_VIDEO_PREFIX "your-vi-prefix")
    set(PLATFORM_MEDIA_CONTROLLER "/dev/media")
    set(PLATFORM_RESET_GPIO "gpio-name-or-number")
    set(PLATFORM_RESET_GPIO_PIN 0)
    set(PLATFORM_RESET_PULSE_US 100000)
    set(PLATFORM_RESET_DELAY_US 5000000)
    message(STATUS "Configured for YOUR_PLATFORM")
```

### Step 2: Implement Platform-Specific Helpers (Optional)

If your platform needs special media configuration (like Raspberry Pi's RP1 CFE), create a subdirectory:

```bash
mkdir libaditof/sdk/src/platform/your_platform/
# Add your_platform_media_config.h/cpp
```

Include it conditionally in `platform_impl.cpp`:
```cpp
#ifdef YOUR_PLATFORM
#include "your_platform/media_config.h"
#endif
```

### Step 3: Update GPIO Handling (If Needed)

If your platform uses a different GPIO interface, add conditional compilation in `platform_impl.cpp::resetSensor()`:

```cpp
#ifdef YOUR_PLATFORM
// Your platform-specific GPIO control
yourPlatformGpioSet(PLATFORM_RESET_GPIO, 0);
usleep(PLATFORM_RESET_PULSE_US);
yourPlatformGpioSet(PLATFORM_RESET_GPIO, 1);
#endif
```

### Step 4: Build and Test

```bash
cmake -DYOUR_PLATFORM=ON ..
make -j$(nproc)

# Test sensor discovery
./examples/first-frame/first-frame
```

### Step 5: Update Documentation

Add your platform to the configuration table in this README.

---

## Integration with SDK

The Platform layer is accessed via `PlatformSensorEnumerator` which implements the SDK's `SensorEnumeratorInterface`:

```cpp
// In SDK initialization (system.cpp or similar)
auto enumerator = std::make_unique<PlatformSensorEnumerator>();
enumerator->searchSensors();

std::vector<std::shared_ptr<DepthSensorInterface>> sensors;
enumerator->getDepthSensors(sensors);

// Use discovered sensors
auto camera = std::make_shared<CameraItof>(sensors[0]);
```

---

## Troubleshooting

### No Sensors Found

**Symptom**: `findToFSensors()` returns empty vector

**Debugging**:
```bash
# 1. Check media devices exist
ls -l /dev/media*

# 2. Manually parse media pipeline
media-ctl -d /dev/media0 -p

# 3. Verify platform capture device name matches
# Should contain PLATFORM_CAPTURE_DEVICE substring
```

**Solution**: Ensure device tree/kernel driver loaded, check `PLATFORM_CAPTURE_DEVICE` matches actual entity name.

### GPIO Reset Fails

**Symptom**: `resetSensor()` returns error

**Debugging**:
```bash
# Check GPIO availability
cat /sys/kernel/debug/gpio  # or ls /dev/gpiochip*

# Check GPIO permissions
ls -l /dev/gpiochip0
```

**Solution**: Ensure GPIO character device accessible, verify `PLATFORM_RESET_GPIO` correct.

### Media Pipeline Format Errors (Raspberry Pi)

**Symptom**: `VIDIOC_STREAMON` fails with format validation error

**Solution**: Ensure `rpi::configureMediaPipeline()` called before camera start.

---

## File Summary

| File | Purpose |
|------|---------|
| `platform.cmake` | CMake configuration (sets platform variables) |
| `platform_config.h.in` | Template for generated config header |
| `platform_impl.h` | Platform class interface |
| `platform_impl.cpp` | Unified platform implementation |
| `sensor_enumerator.h/cpp` | SDK integration bridge |
| `raspberrypi/rpi_media_config.h/cpp` | Raspberry Pi RP1 CFE helpers |
