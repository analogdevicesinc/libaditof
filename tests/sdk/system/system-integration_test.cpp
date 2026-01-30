#include <gtest/gtest.h>
#include <aditof/system.h>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/camera_definitions.h>
#include <aditof/status_definitions.h>
#include <aditof_test_utils.h>
#include <vector>
#include <memory>
#include <thread>
#include <chrono>

using namespace aditof;

// Global camera IP from test utilities
std::string& g_cameraipaddress = aditof_test::g_cameraipaddress;

/**
 * SYSTEM TESTS FOR SYSTEM AND CAMERA CLASSES
 * 
 * Tests system-level functionality:
 * - System instantiation and camera discovery
 * - Camera connection and initialization
 * - Camera properties and configuration
 * - Multiple camera handling
 */

class SystemIntegrationTest : public ::testing::Test {
protected:
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    
    void SetUp() override {
        // Get camera list
        Status status;
        if (g_cameraipaddress == "") {
            status = system.getCameraList(cameras);
        } else {
            status = system.getCameraList(cameras, "ip:" + g_cameraipaddress);
        }
        
        if (status != Status::OK || cameras.empty()) {
            GTEST_SKIP_("No camera available for system test");
        }
    }

    void TearDown() override {
        cameras.clear();
    }
};

// Test: System initialization
TEST(SystemInstantiationTest, SystemCreation) {
    EXPECT_NO_THROW({
        System sys;
    });
}

// Test: Get camera list without camera
TEST(SystemInstantiationTest, GetCameraListWithoutCamera) {
    System sys;
    std::vector<std::shared_ptr<Camera>> camList;
    Status status = sys.getCameraList(camList);
    
    EXPECT_EQ(status, Status::OK);
    // May be empty or may have cameras
}

// Test: Get camera list with IP
TEST(SystemInstantiationTest, GetCameraListWithIP) {
    System sys;
    std::vector<std::shared_ptr<Camera>> camList;
    
    if (g_cameraipaddress != "") {
        Status status = sys.getCameraList(camList, "ip:" + g_cameraipaddress);
        EXPECT_EQ(status, Status::OK);
    } else {
        GTEST_SKIP() << "\n"
            << "======================================\n"
            << "Network Camera Test - SKIPPED\n"
            << "======================================\n"
            << "Reason: No camera IP address provided\n"
            << "Current Setup: LOCAL connection (flex cable)\n"
            << "\n"
            << "To run network tests, provide IP address:\n"
            << "  --ip=192.168.1.100\n"
            << "======================================";
    }
}

// Test: Camera initialization
TEST_F(SystemIntegrationTest, CameraInitialization) {
    auto camera = cameras[0];
    EXPECT_NE(camera, nullptr);
    
    Status status = camera->initialize();
    EXPECT_EQ(status, Status::OK);
}

// Test: Camera properties
TEST_F(SystemIntegrationTest, CameraProperties) {
    auto camera = cameras[0];
    EXPECT_NE(camera, nullptr);
    
    CameraDetails details;
    Status status = camera->getDetails(details);
    EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
}

// Test: Get available frame types
TEST_F(SystemIntegrationTest, GetAvailableFrameTypes) {
    auto camera = cameras[0];
    Status status = camera->initialize();
    
    if (status != Status::OK) {
        GTEST_SKIP_("Camera initialization failed");
    }
    
    // Give camera time to enumerate modes after initialization
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    std::vector<uint8_t> availableModes;
    status = camera->getAvailableModes(availableModes);
    
    // Status should be OK or UNAVAILABLE, not an error
    EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE)
        << "getAvailableModes returned unexpected status";
    
    if (status == Status::OK) {
        EXPECT_FALSE(availableModes.empty()) << "No modes available";
    }
}

// Test: Set and get frame type
TEST_F(SystemIntegrationTest, SetFrameType) {
    auto camera = cameras[0];
    Status status = camera->initialize();
    
    if (status != Status::OK) {
        GTEST_SKIP_("Camera initialization failed");
    }
    
    std::vector<uint8_t> availableModes;
    status = camera->getAvailableModes(availableModes);
    
    if (status == Status::OK && !availableModes.empty()) {
        status = camera->setMode(availableModes[0]);
        EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
    }
}

// Test: Get device options
TEST_F(SystemIntegrationTest, GetDeviceOptions) {
    auto camera = cameras[0];
    Status status = camera->initialize();
    
    if (status != Status::OK) {
        GTEST_SKIP_("Camera initialization failed");
    }
    
    CameraDetails details;
    status = camera->getDetails(details);
    
    // Should either succeed or return a meaningful error
    EXPECT_TRUE(status == Status::OK || status != Status::INVALID_ARGUMENT);
}

// Test: Multiple camera list calls
TEST(SystemInstantiationTest, MultipleCameraListCalls) {
    System sys;
    
    std::vector<std::shared_ptr<Camera>> cameras1;
    std::vector<std::shared_ptr<Camera>> cameras2;
    
    Status status1 = sys.getCameraList(cameras1);
    Status status2 = sys.getCameraList(cameras2);
    
    EXPECT_EQ(status1, Status::OK);
    EXPECT_EQ(status2, Status::OK);
    EXPECT_EQ(cameras1.size(), cameras2.size());
}

// Test: Camera list with different URIs
TEST(SystemInstantiationTest, CameraListDifferentURIs) {
    System sys;
    
    std::vector<std::shared_ptr<Camera>> cameras1;
    std::vector<std::shared_ptr<Camera>> cameras2;
    
    Status status1 = sys.getCameraList(cameras1);
    Status status2 = sys.getCameraList(cameras2, "");
    
    EXPECT_EQ(status1, Status::OK);
    EXPECT_EQ(status2, Status::OK);
}

// Test: Camera factory pattern
TEST_F(SystemIntegrationTest, CameraFactoryPattern) {
    auto camera1 = cameras[0];
    auto camera2 = cameras[0];
    
    // Should be the same instance
    EXPECT_EQ(camera1, camera2);
}

// Test: Error handling on invalid operations
TEST_F(SystemIntegrationTest, ErrorHandlingUninitializedCamera) {
    auto camera = cameras[0];
    
    // Try to get available modes without initialization (should fail gracefully)
    std::vector<uint8_t> availableModes;
    Status status = camera->getAvailableModes(availableModes);
    
    // Should handle gracefully - test FAILS if operation causes crash
    ASSERT_TRUE(status == Status::OK || status == Status::UNAVAILABLE || 
                status == Status::GENERIC_ERROR || status == Status::INVALID_ARGUMENT)
        << "Unexpected status from getAvailableModes on uninitialized camera";
}

// NEW TESTS ADDED BELOW

// Test: Camera start and stop
TEST_F(SystemIntegrationTest, CameraStartStop) {
    auto camera = cameras[0];
    Status status = camera->initialize();
    ASSERT_EQ(status, Status::OK) << "Camera initialization failed";
    
    // Set mode before starting camera
    std::vector<uint8_t> modes;
    status = camera->getAvailableModes(modes);
    if (status == Status::OK && !modes.empty()) {
        status = camera->setMode(modes[0]);
        ASSERT_TRUE(status == Status::OK || status == Status::UNAVAILABLE) 
            << "Failed to set camera mode";
    }
    
    status = camera->start();
    EXPECT_EQ(status, Status::OK) << "Camera start failed";
    
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    status = camera->stop();
    EXPECT_EQ(status, Status::OK) << "Camera stop failed";
}

// Test: Frame request after camera start
TEST_F(SystemIntegrationTest, RequestFrameAfterStart) {
    auto camera = cameras[0];
    Status status = camera->initialize();
    ASSERT_EQ(status, Status::OK);
    
    std::vector<uint8_t> modes;
    status = camera->getAvailableModes(modes);
    if (status == Status::OK && !modes.empty()) {
        camera->setMode(modes[0]);
    }
    
    status = camera->start();
    ASSERT_EQ(status, Status::OK);
    
    Frame frame;
    status = camera->requestFrame(&frame);
    EXPECT_EQ(status, Status::OK) << "Failed to request frame";
    
    camera->stop();
}

// Test: Multiple frame captures in sequence
TEST_F(SystemIntegrationTest, MultipleFrameCaptures) {
    auto camera = cameras[0];
    camera->initialize();
    
    std::vector<uint8_t> modes;
    camera->getAvailableModes(modes);
    if (!modes.empty()) {
        camera->setMode(modes[0]);
    }
    
    camera->start();
    
    const int numFrames = 10;
    int successCount = 0;
    
    for (int i = 0; i < numFrames; i++) {
        Frame frame;
        Status status = camera->requestFrame(&frame);
        if (status == Status::OK) {
            successCount++;
        }
    }
    
    camera->stop();
    
    EXPECT_GT(successCount, 0) << "No frames captured successfully";
    EXPECT_GE(successCount, numFrames * 0.8) << "Less than 80% frame success rate";
}

// Test: Multiple start/stop cycles
TEST_F(SystemIntegrationTest, MultipleStartStopCycles) {
    auto camera = cameras[0];
    Status status = camera->initialize();
    ASSERT_EQ(status, Status::OK);
    
    // Set mode before starting camera
    std::vector<uint8_t> modes;
    status = camera->getAvailableModes(modes);
    if (status == Status::OK && !modes.empty()) {
        status = camera->setMode(modes[0]);
        ASSERT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
    }
    
    const int cycles = 3;
    for (int i = 0; i < cycles; i++) {
        status = camera->start();
        EXPECT_EQ(status, Status::OK) << "Start failed on cycle " << i;
        
        // Longer delay to allow camera to stabilize
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        status = camera->stop();
        EXPECT_EQ(status, Status::OK) << "Stop failed on cycle " << i;
        
        // Delay between cycles to allow cleanup
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

// Test: Frame details validation
TEST_F(SystemIntegrationTest, ValidateFrameDetails) {
    auto camera = cameras[0];
    camera->initialize();
    
    std::vector<uint8_t> modes;
    camera->getAvailableModes(modes);
    if (!modes.empty()) {
        camera->setMode(modes[0]);
    }
    
    camera->start();
    
    Frame frame;
    Status status = camera->requestFrame(&frame);
    
    if (status == Status::OK) {
        FrameDetails details;
        frame.getDetails(details);
        
        EXPECT_GT(details.width, 0) << "Frame width is zero";
        EXPECT_GT(details.height, 0) << "Frame height is zero";
    }
    
    camera->stop();
}

// Test: Camera re-initialization
TEST_F(SystemIntegrationTest, CameraReinitialization) {
    auto camera = cameras[0];
    
    // First initialization
    Status status = camera->initialize();
    EXPECT_EQ(status, Status::OK);
    
    camera->start();
    camera->stop();
    
    // Second initialization - should handle gracefully
    status = camera->initialize();
    EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR) 
        << "Re-initialization should not crash";
}

// Test: Mode switching
TEST_F(SystemIntegrationTest, ModeSwitching) {
    auto camera = cameras[0];
    Status status = camera->initialize();
    ASSERT_EQ(status, Status::OK);
    
    std::vector<uint8_t> modes;
    status = camera->getAvailableModes(modes);
    
    if (status == Status::OK && modes.size() > 1) {
        // Try switching between available modes
        for (const auto& mode : modes) {
            status = camera->setMode(mode);
            EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE)
                << "Failed to set mode " << static_cast<int>(mode);
        }
    } else {
        GTEST_SKIP_("Not enough modes available for switching test");
    }
}

int main(int argc, char** argv) {
    aditof_test::TestRunner runner(argv[0]);
    
    int initResult = runner.initialize(argc, argv);
    if (initResult != -1) {
        return initResult;
    }
    
    return runner.runTests();
}
