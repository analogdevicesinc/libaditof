#include <gtest/gtest.h>
#include <aditof/system.h>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/status_definitions.h>
#include <aditof/camera_definitions.h>
#include <aditof/utils.h>
#include <aditof_test_utils.h>
#include <vector>
#include <memory>
#include <string>

using namespace aditof;

// Global camera IP
std::string& g_cameraipaddress = aditof_test::g_cameraipaddress;

/**
 * SDK SOURCE FILES INTEGRATION TESTS
 * 
 * Tests that source files work correctly together:
 * - system.cpp - System class implementation
 * - frame.cpp - Frame class implementation
 * - status_definitions.cpp - Status handling
 * - utils.cpp - Utility functions
 * - frame_handler.cpp - Frame handling
 * - frame_operations.cpp - Frame operations
 */

class SDKSourceIntegrationTest : public ::testing::Test {
protected:
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    
    void SetUp() override {
        Status status;
        if (g_cameraipaddress == "") {
            status = system.getCameraList(cameras);
        } else {
            status = system.getCameraList(cameras, "ip:" + g_cameraipaddress);
        }
        
        if (status != Status::OK || cameras.empty()) {
            GTEST_SKIP_("No camera available");
        }
    }

    void TearDown() override {
        cameras.clear();
    }
};

// Test: System initialization from system.cpp
TEST(SystemCppIntegrationTest, SystemImplementation) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    
    Status status = system.getCameraList(cameras);
    EXPECT_EQ(status, Status::OK);
}

// Test: Status handling from status_definitions.cpp
TEST(StatusCppIntegrationTest, StatusImplementation) {
    Status status1 = Status::OK;
    Status status2 = Status::BUSY;
    Status status3 = Status::INVALID_ARGUMENT;
    
    EXPECT_NE(status1, status2);
    EXPECT_NE(status2, status3);
    
    // Test output stream (implemented in status_definitions.cpp)
    std::ostringstream oss;
    oss << status1;
    EXPECT_FALSE(oss.str().empty());
}

// Test: Frame implementation from frame.cpp
TEST(FrameCppIntegrationTest, FrameImplementation) {
    Frame frame;
    FrameDetails details;
    
    details.width = 512;
    details.height = 512;
    details.type = "depth";
    details.cameraMode = "near";
    
    Status status = frame.getDetails(details);
    EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
}

// Test: Utils implementation from utils.cpp
TEST(UtilsCppIntegrationTest, UtilsImplementation) {
    std::string input = "test,path,components";
    std::vector<std::string> tokens;
    
    Utils::splitIntoTokens(input, ',', tokens);
    EXPECT_EQ(tokens.size(), 3);
    
    std::string execPath = Utils::getExecutableFolder();
    EXPECT_FALSE(execPath.empty());
}

// Test: Frame and System integration
TEST_F(SDKSourceIntegrationTest, FrameAndSystemIntegration) {
    auto camera = cameras[0];
    
    Status status = camera->initialize();
    EXPECT_EQ(status, Status::OK);
    
    Frame frame;
    FrameDetails details;
    details.width = 512;
    details.height = 512;
    details.type = "depth";
    details.cameraMode = "near";
    
    status = frame.getDetails(details);
    EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
}

// Test: Status propagation through APIs
TEST_F(SDKSourceIntegrationTest, StatusPropagation) {
    auto camera = cameras[0];
    
    // Get camera details - tests status propagation
    CameraDetails cameraDetails;
    Status status = camera->getDetails(cameraDetails);
    
    // Should return a valid status
    EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
}

// Test: Frame handling workflow
TEST_F(SDKSourceIntegrationTest, FrameHandlingWorkflow) {
    auto camera = cameras[0];
    
    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP_("Camera init failed");
    }
    
    // Create frames
    Frame frame1;
    Frame frame2;
    
    FrameDetails details;
    details.width = 512;
    details.height = 512;
    details.type = "depth";
    details.cameraMode = "near";
    
    status = frame1.getDetails(details);
    EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
    
    // Copy frame
    frame2 = frame1;
    try {
        frame2.getDetails(details);
        EXPECT_TRUE(true);
    } catch (...) {
        FAIL() << "Frame copy or getDetails threw unexpected exception";
    }
}

// Test: Utils used in frame configuration
TEST_F(SDKSourceIntegrationTest, UtilsInFrameConfig) {
    // Simulate parsing frame configuration
    std::string config = "512x512:depth:near";
    std::vector<std::string> parts;
    
    Utils::splitIntoTokens(config, ':', parts);
    EXPECT_EQ(parts.size(), 3);
    
    // Create frame from parsed config
    Frame frame;
    FrameDetails details;
    details.type = "depth";
    details.cameraMode = "near";
    
    Status status = frame.getDetails(details);
    EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
}

// Test: Camera list processing with utils
TEST(SystemUtilsIntegrationTest, CameraListProcessing) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    
    Status status = system.getCameraList(cameras);
    EXPECT_EQ(status, Status::OK);
    
    // Process camera info using utils
    for (auto& camera : cameras) {
        CameraDetails details;
        status = camera->getDetails(details);
        
        // Could use utils to parse camera info
        std::vector<std::string> parts;
        if (!details.cameraId.empty()) {
            Utils::splitIntoTokens(details.cameraId, '-', parts);
            EXPECT_GE(parts.size(), 1);
        } else {
            EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
        }
    }
}

// Test: Error handling across files
TEST(ErrorHandlingIntegrationTest, ErrorHandlingAcrossFiles) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    
    // Status from system.cpp
    Status status = system.getCameraList(cameras);
    EXPECT_EQ(status, Status::OK);
    
    if (!cameras.empty()) {
        auto camera = cameras[0];
        
        // Status propagation from camera.cpp
        status = camera->initialize();
        EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
        
        // Status in frame.cpp
        Frame frame;
        FrameDetails details;
        details.width = 512;
        details.height = 512;
        details.type = "depth";
        details.cameraMode = "near";
        
        status = frame.getDetails(details);
        EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
    }
}

// Test: Multiple frame operations
TEST(FrameOperationsIntegrationTest, MultipleFrameOperations) {
    std::vector<Frame> frames;
    
    // Create multiple frames
    for (int i = 0; i < 5; i++) {
        Frame frame;
        FrameDetails details;
        
        details.width = 512;
        details.height = 512;
        details.type = "depth";
        details.cameraMode = "near";
        
        Status status = frame.getDetails(details);
        EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
        
        frames.push_back(std::move(frame));
    }
    
    EXPECT_EQ(frames.size(), 5);
}

// Test: System and frame details integration
TEST(SystemFrameIntegrationTest, SystemFrameDetails) {
    System system;
    Frame frame;
    
    // Get camera list
    std::vector<std::shared_ptr<Camera>> cameras;
    Status status = system.getCameraList(cameras);
    EXPECT_EQ(status, Status::OK);
    
    // Get frame details
    FrameDetails details;
    details.width = 512;
    details.height = 512;
    details.type = "depth";
    details.cameraMode = "near";
    
    status = frame.getDetails(details);
    EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
}

// Test: Full workflow - System → Camera → Frame
TEST_F(SDKSourceIntegrationTest, FullWorkflow) {
    auto camera = cameras[0];
    
    // Initialize camera
    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP_("Camera init failed");
    }
    
    // Get available modes
    std::vector<uint8_t> availableModes;
    status = camera->getAvailableModes(availableModes);
    EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
    
    // Create and configure frame
    Frame frame;
    FrameDetails details;
    details.width = 512;
    details.height = 512;
    details.type = "depth";
    details.cameraMode = "near";
    
    status = frame.getDetails(details);
    EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
}

int main(int argc, char** argv) {
    aditof_test::TestRunner runner(argv[0]);
    
    int initResult = runner.initialize(argc, argv);
    if (initResult != -1) {
        return initResult;
    }
    
    return runner.runTests();
}
