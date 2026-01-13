### Setting Up for testing the SDK using GoogleTests

In the *libaditof* folder:
```
git submodule update --init gtest
mkdir build && cd build
cmake -DBUILD_TESTING=ON -DNVIDIA=1 ..
cmake --build . --config RELEASE -j 6
```

Running a test:
```
$ ./generic-sdk_version_test --gtest_output=json:generic-sdk_version_test.json
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

$ more generic-sdk_version_test.json
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

Running a test where a failure occurs:
```
$ ./generic-sdk_version_test --version=2.3.4 --gtest_output=json:generic-sdk_version_test.json
[==========] Running 2 tests from 1 test suite.
[----------] Global test environment set-up.
[----------] 2 tests from VersionTest
[ RUN      ] VersionTest.VersionMacrosAreDefined
[       OK ] VersionTest.VersionMacrosAreDefined (0 ms)
[ RUN      ] VersionTest.ApiVersionReturnsNonEmptyString
/home/analog/dev/ADCAM.gtest/libaditof/tests/sdk/generic/generic-sdk_version_test.cpp:35: Failure
Value of: version == g_expectedVersion
  Actual: false
Expected: true

[  FAILED  ] VersionTest.ApiVersionReturnsNonEmptyString (0 ms)
[----------] 2 tests from VersionTest (0 ms total)

[----------] Global test environment tear-down
[==========] 2 tests from 1 test suite ran. (0 ms total)
[  PASSED  ] 1 test.
[  FAILED  ] 1 test, listed below:
[  FAILED  ] VersionTest.ApiVersionReturnsNonEmptyString

 1 FAILED TEST
 $ more generic-sdk_version_test.json
{
  "tests": 2,
  "failures": 1,
  "disabled": 0,
  "errors": 0,
  "timestamp": "2026-01-09T08:37:14Z",
  "time": "0s",
  "name": "AllTests",
  "testsuites": [
    {
      "name": "VersionTest",
      "tests": 2,
      "failures": 1,
      "disabled": 0,
      "errors": 0,
      "timestamp": "2026-01-09T08:37:14Z",
      "time": "0s",
      "testsuite": [
        {
          "name": "VersionMacrosAreDefined",
          "file": "\/home\/analog\/dev\/ADCAM.gtest\/libaditof\/tests\/sdk\/generic\/generic-sdk_version_test.cpp",
          "line": 18,
          "status": "RUN",
          "result": "COMPLETED",
          "timestamp": "2026-01-09T08:37:14Z",
          "time": "0s",
          "classname": "VersionTest"
        },
        {
          "name": "ApiVersionReturnsNonEmptyString",
          "file": "\/home\/analog\/dev\/ADCAM.gtest\/libaditof\/tests\/sdk\/generic\/generic-sdk_version_test.cpp",
          "line": 29,
          "status": "RUN",
          "result": "COMPLETED",
          "timestamp": "2026-01-09T08:37:14Z",
          "time": "0s",
          "classname": "VersionTest",
          "failures": [
            {
              "failure": "\/home\/analog\/dev\/ADCAM.gtest\/libaditof\/tests\/sdk\/generic\/generic-sdk_version_test.cpp:35\nV
alue of: version == g_expectedVersion\n  Actual: false\nExpected: true\n",
              "type": ""
            }
          ]
        }
      ]
    }
  ]
} 
```
