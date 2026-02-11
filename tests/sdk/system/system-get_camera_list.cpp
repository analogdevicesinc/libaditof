#include <aditof/camera.h>
#include <aditof/camera_definitions.h>
#include <aditof/depth_sensor_interface.h>
#include <aditof/frame.h>
#include <aditof/status_definitions.h>
#include <aditof/system.h>
#include <aditof/version.h>
#include <aditof_test_utils.h>
#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <thread>
#include <vector>

using namespace aditof;

// Use the camera IP address from aditof_test library
std::string &g_cameraipaddress = aditof_test::g_cameraipaddress;

// Test System class
TEST(SystemTest, SystemInstantiation) {
    EXPECT_NO_THROW({ System system; });
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

    ASSERT_EQ(status, Status::OK) << "Failed to get camera list";
   
    ASSERT_FALSE(cameras.empty()) << "No cameras detected - check if ADCAM is connected";
    
    if (!cameras.empty()) {
        auto camera = cameras.front();
        EXPECT_NE(camera, nullptr) << "Camera pointer is null";
    }
}

// Test: Local camera detection (flex cable)
TEST(SystemTest, LocalCameraDetection) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    
    // Force local discovery (ignoring IP if provided)
    Status status = system.getCameraList(cameras);
    EXPECT_EQ(status, Status::OK);
    
    if (g_cameraipaddress.empty()) {
        // Should find camera via flex cable
        EXPECT_FALSE(cameras.empty()) << "No local camera found via flex cable";
    }
}

// Test: Camera list not empty
TEST(SystemTest, CameraListNotEmpty) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    Status status;
    
    if (g_cameraipaddress == "") {
        status = system.getCameraList(cameras);
    } else {
        status = system.getCameraList(cameras, "ip:" + g_cameraipaddress);
    }
    
    ASSERT_EQ(status, Status::OK);
    ASSERT_GT(cameras.size(), 0) << "Camera list is empty - ADCAM not detected";
}

// Test: Camera pointer validity
TEST(SystemTest, CameraPointerValid) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    Status status;
    
    if (g_cameraipaddress == "") {
        status = system.getCameraList(cameras);
    } else {
        status = system.getCameraList(cameras, "ip:" + g_cameraipaddress);
    }
    
    ASSERT_EQ(status, Status::OK);
    ASSERT_FALSE(cameras.empty());
    
    for (size_t i = 0; i < cameras.size(); i++) {
        EXPECT_NE(cameras[i], nullptr) << "Camera " << i << " pointer is null";
    }
}

// Test: Multiple camera list calls consistency
TEST(SystemTest, MultipleCameraListCalls) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras1, cameras2;
    
    Status status1, status2;
    if (g_cameraipaddress == "") {
        status1 = system.getCameraList(cameras1);
        status2 = system.getCameraList(cameras2);
    } else {
        status1 = system.getCameraList(cameras1, "ip:" + g_cameraipaddress);
        status2 = system.getCameraList(cameras2, "ip:" + g_cameraipaddress);
    }
    
    EXPECT_EQ(status1, Status::OK);
    EXPECT_EQ(status2, Status::OK);
    EXPECT_EQ(cameras1.size(), cameras2.size()) << "Camera count inconsistent between calls";
}

// Test: Connection type detection
TEST(SystemTest, ConnectionTypeDetection) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    Status status;
    
    if (g_cameraipaddress == "") {
        std::cout << "Testing LOCAL connection (flex cable)" << std::endl;
        status = system.getCameraList(cameras);
    } else {
        std::cout << "Testing NETWORK connection (IP: " << g_cameraipaddress << ")" << std::endl;
        status = system.getCameraList(cameras, "ip:" + g_cameraipaddress);
    }
    
    ASSERT_EQ(status, Status::OK);
    ASSERT_FALSE(cameras.empty()) << "No camera detected for current connection type";
}

int main(int argc, char** argv) {
    // Create test runner
    aditof_test::TestRunner runner(argv[0]);

    // Note: --ip argument is automatically added by TestRunner

    // Initialize (parses args, sets up GTest output)
    int initResult = runner.initialize(argc, argv);
    if (initResult != -1) {
        return initResult; // Help was shown or error occurred
    }

    // Run tests
    return runner.runTests();
}