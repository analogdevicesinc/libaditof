#include <gtest/gtest.h>
#include <aditof/version.h>
#include <aditof/system.h>
#include <aditof_test_utils.h>
#include <iostream>
#include <vector>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/camera_definitions.h>
#include <aditof/status_definitions.h>
#include <aditof/depth_sensor_interface.h>
#include <array>
#include <cmath>
#include <thread>

using namespace aditof;

// Use the camera IP address from aditof_test library
std::string& g_cameraipaddress = aditof_test::g_cameraipaddress;

// Test System class
TEST(SystemTest, SystemInstantiation) {
    EXPECT_NO_THROW({
        System system;
    });
}

TEST(SystemTest, GetCameraListCameras) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    Status status;
    // This should not throw even if no cameras are connected
    EXPECT_NO_THROW({
        if (g_cameraipaddress == "") {
            status = system.getCameraList(cameras);
        } else {
            status = system.getCameraList(cameras, "ip:" + g_cameraipaddress);
        }
    });

    ASSERT_TRUE(status == Status::OK);
   
    bool has_camera = false;
    if (!cameras.empty()) {
        auto camera = cameras.front();
        has_camera = true;
    } else {
        has_camera = false;
    }

    ASSERT_TRUE(has_camera);
}

int main(int argc, char** argv) {
    // Create test runner
    aditof_test::TestRunner runner(argv[0]);
    
    // Note: --ip argument is automatically added by TestRunner
    
    // Initialize (parses args, sets up GTest output)
    int initResult = runner.initialize(argc, argv);
    if (initResult != -1) {
        return initResult;  // Help was shown or error occurred
    }
    
    // Run tests
    return runner.runTests();
}