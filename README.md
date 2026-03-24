# Analog Devices 3D ToF SDK 

Please note, the [libaditof GitHub Wiki](https://github.com/analogdevicesinc/libaditof/wiki) is available with additional documentation.

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

Note: all code must be formatted by clang-format and issues via clang-tidy must be addressed.

See the documents:

* [code-formatting.md](doc/code-formatting.md)
* [code-style.md](doc/code-style.md)

### Linux Build

#### Pre-requisites
* CMake
* g++
* Python 3 - note, we are assuming Python 3.10 in this document, change as needed for your setup
* Doxygen - for documentation generation
* Graphviz - for documentation generation

#### Installing the pre-requisites

Note, *python3.10-dev* is specific to the NVIDIA Jetson Orin Nano Dev Kit Jetpack 6.2.

If your Linux does not use Python 3.10 you will need to change *python3.10-dev* to reflect the version of Linux on your system.

```console
sudo apt update
sudo apt install cmake g++ \
     doxygen graphviz \
     python3.10-dev
```

#### Dependencies

In addition the depth compute libraries are required. 

You can get the two library from the ADCAM release software, but please note in which case it is under an evaluation license.

For a non-eval license please contact us at *tof@analog.com*.

These libraries must be in a folder called **libs** that in one level below the cloned ADCAM repo folder. For example:
```
(aditofpython_env) analog@analog-desktop:~/dev/ADCAM$ pwd
/home/analog/dev/ADCAM
(aditofpython_env) analog@analog-desktop:~/dev/ADCAM$ tree ../libs
../libs
├── libtofi_compute.so
└── libtofi_config.so
```

```
git clone https://github.com/analogdevicesinc/libaditof
cd libaditof
git submodule update --init
git checkout <branch or tag>
mkdir build && cd build
cmake -DNVIDIA=1 ..
cmake --build . --config RELEASE -j 6
```

SDK binary and associated softlinks are hare **build/libaditof.so* **

### Windows Build

See [windows-build.md](doc/windows-build.md).

## CMake Options

For the build options in [CMakeLists.txt](./CMakeLists.txt) see [cmake/readme.md](./cmake/readme.md).

## Tests

Test infrastructure using GoogleTests and PyTest has been added.

See [tests/README.md](./tests/README.md).
