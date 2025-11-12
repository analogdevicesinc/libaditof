# Analog Devices 3D ToF SDK 

## Overview

The ADI ToF SDK is designed to support ADI's ToF hardware.

```
ToF Imager (ADSD3100 or ADSD3030)
            ||
            ||
    ADSD3500 (Dual or Single)
            ||
            || (MIPI for Frame Data)
            || (I2C or SPI for Control)
            ||
------------------------------------------
            |                      (Linux) 
       Host Driver
            |
            |
         libaditof


|| - Hardware Connection
 | - Software Connection
```

The SDK provides an API to control the ToF camera, AB, and depth frame streams.

License : [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Repos using libaditof:

* [ADTF3175D Eval Kit](https://github.com/analogdevicesinc/ToF)
* [ADCAM3175 Camera Kit](https://github.com/analogdevicesinc/ADCAM)

**Note, prior to committing to the repo it is important to format the source code, see the [code formatting](./doc/code-formatting.md).**

## Build libaditof SDK

### Linux Build

```
git clone https://github.com/analogdevicesinc/libaditof
cd libaditof
git submodule update --init
git checkout <branch or tag>
mkdir build && cd build
cmake -DNVIDIA=1 ..
make -j"$(nproc)"
```

SDK binary and associated softlinks are hare **build/libaditof.so* **

### Windows Build

```
git clone https://github.com/analogdevicesinc/libaditof
cd libaditof
git submodule update --init
git checkout <branch or tag>
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release -G "Visual Studio 17 2022" ..
cmake --build . --config RELEASE -j 8
```

## CMake Options

For the build options in [CMakeLists.txt](./CMakeLists.txt) see [cmake/readme.md](./cmake/readme.md).
