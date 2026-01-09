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

#### Dependencies

In addition the depth compute libraries are required. 

You can get the two library from the ADCAM release software, but please note in which case it is under an evaluation license.

For a non-eval license please contact us at *tof@analog.com*.

These libraries must be in a folder called **libs** that in one level below the cloned ADCAM repo folder. For example:
```
c:\dev\ADCAM> tree ../libs
../libs
├── libtofi_compute.dll
└── libtofi_config.dll
```

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

## Tests

Test infrastructure using GoogleTests and PyTest has been added.

### Setting Up for testing the SDK using GoogleTests

In the *libaditof* folder:
```
git submodule update --init extern/gtest
mkdir build && cd build
cmake -DBUILD_TESTING=ON -DNVIDIA=1 ..
cmake --build . --config RELEASE -j 6
```

Running a test:
```
$> ./generic-sdk_version_test --gtest_output=json:generic-sdk_version_test.json
[==========] Running 2 tests from 1 test suite.
[----------] Global test environment set-up.
[----------] 2 tests from VersionTest
[ RUN      ] VersionTest.VersionMacrosAreDefined
[       OK ] VersionTest.VersionMacrosAreDefined (0 ms)
[ RUN      ] VersionTest.ApiVersionReturnsNonEmptyString
[       OK ] VersionTest.ApiVersionReturnsNonEmptyString (0 ms)
[----------] 2 tests from VersionTest (0 ms total)

[----------] Global test environment tear-down
[==========] 2 tests from 1 test suite ran. (0 ms total)
[  PASSED  ] 2 tests.

$> more generic-sdk_version_test.json
{
  "tests": 2,
  "failures": 0,
  "disabled": 0,
  "errors": 0,
  "timestamp": "2026-01-09T07:36:02Z",
  "time": "0s",
  "name": "AllTests",
  "testsuites": [
    {
      "name": "VersionTest",
      "tests": 2,
      "failures": 0,
      "disabled": 0,
      "errors": 0,
      "timestamp": "2026-01-09T07:36:02Z",
      "time": "0s",
      "testsuite": [
        {
          "name": "VersionMacrosAreDefined",
          "file": "\/home\/analog\/dev\/ADCAM.gtest\/libaditof\/tests\/sdk\/generic\/generic-sdk_version_test.cpp",
          "line": 18,
          "status": "RUN",
          "result": "COMPLETED",
          "timestamp": "2026-01-09T07:36:02Z",
          "time": "0s",
          "classname": "VersionTest"
        },
        {
          "name": "ApiVersionReturnsNonEmptyString",
          "file": "\/home\/analog\/dev\/ADCAM.gtest\/libaditof\/tests\/sdk\/generic\/generic-sdk_version_test.cpp",
          "line": 29,
          "status": "RUN",
          "result": "COMPLETED",
          "timestamp": "2026-01-09T07:36:02Z",
          "time": "0s",
          "classname": "VersionTest"
        }
      ]
    }
  ]
}
```