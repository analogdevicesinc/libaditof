# Setting Up for testing the SDK using GoogleTests

## Developing a test:

### Building the repo
```
git submodule update --init gtest
mkdir build && cd build
cmake -DBUILD_TESTING=ON -DNVIDIA=1 -DWITH_NETWORK=OFF -DWITH_SUBMODULES=OFF ..
make -j6
```

### Running a test from the build folder
```
$ cd build/tests/sdk/bin
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

## Using the test environment

### Using start_tests.sh

For example:
```
$ cd tests
$ ./start_tests.sh -f test_csvs/sample_test_list.csv -o ~/Documents/
$ cd ~/Documents
$ ls
results_sample_test_list_20260122_110837.sh
$ ./results_sample_test_list_20260122_110837.sh
Test executing: ./run_tests.sh -f /media/analog/43a17749-484a-4b16-8532-036d47fe816f/home/astraker/dev/ADCAM.gtest/libaditof/tests/test_csvs/sample_test_list.csv -o /tmp/results_sample_test_list_20260122_110837 -n 1

--- Processing ---
GE000000,generic-sdk_version_test,,run_1,Passed
CA000000,camera-adsd3500_reset,,run_1,Passed
CA002000,camera-parameters,--module=crosby,run_1,Passed
--- Summary of Test Results in '/tmp/results_sample_test_list_20260122_110837' ---
Processed,3
Skipped,0
Passed,3
Failed,0
$ ./results_sample_test_list_20260122_110837.sh -x
Extracted to results_sample_test_list_20260122_110837.tar.gz
Unarchived to results_sample_test_list_20260122_110837
$ cd results_sample_test_list_20260122_110837
$ ls
docker_summary.log  libaditof_summary.log  test_CA000000  test_CA002000  test_GE000000
```