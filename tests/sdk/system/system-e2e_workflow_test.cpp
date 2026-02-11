/**
 * SYSTEM TEST: End-to-End Camera Workflow
 * 
 * Tests the complete ToF camera pipeline:
 * System -> getCameraList -> Camera -> initialize -> setMode -> start -> requestFrame -> stop
 * 
 * This tests the full integration of all SDK components working together.
 */

#include <gtest/gtest.h>
#include <aditof/system.h>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/camera_definitions.h>
#include <aditof/frame_definitions.h>
#include <aditof/status_definitions.h>
#include <aditof_test_utils.h>
#include <vector>
#include <memory>
#include <chrono>

using namespace aditof;

std::string& g_cameraipaddress = aditof_test::g_cameraipaddress;

/**
 * E2E_CameraWorkflow - Tests complete camera operation from discovery to frame capture
 */
class E2ECameraWorkflowTest : public ::testing::Test {
protected:
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    std::shared_ptr<Camera> camera;
    
    void SetUp() override {
        Status status;
        if (g_cameraipaddress.empty()) {
            status = system.getCameraList(cameras);
        } else {
            status = system.getCameraList(cameras, "ip:" + g_cameraipaddress);
        }
        
        if (status != Status::OK || cameras.empty()) {
            GTEST_SKIP_("No camera available for E2E workflow test");
        }
        camera = cameras[0];
    }
    
    void TearDown() override {
        if (camera) {
            camera->stop();
        }
    }
};

// E2E Test: Discovery -> Initialize -> GetDetails
TEST_F(E2ECameraWorkflowTest, DiscoveryToInitializeToDetails) {
    // Step 1: Camera already discovered in SetUp
    ASSERT_NE(camera, nullptr);
    
    // Step 2: Initialize
    Status status = camera->initialize();
    ASSERT_EQ(status, Status::OK) << "Camera initialize failed";
    
    // Step 3: Get details after initialization
    CameraDetails details;
    status = camera->getDetails(details);
    EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
}

// E2E Test: Initialize -> GetModes -> SetMode
TEST_F(E2ECameraWorkflowTest, InitializeToModeSelection) {
    // Initialize
    Status status = camera->initialize();
    ASSERT_EQ(status, Status::OK);
    
    // Get available modes
    std::vector<uint8_t> availableModes;
    status = camera->getAvailableModes(availableModes);
    ASSERT_EQ(status, Status::OK) << "getAvailableModes failed";
    ASSERT_FALSE(availableModes.empty()) << "No modes available";
    
    // Set first available mode
    status = camera->setMode(availableModes[0]);
    EXPECT_EQ(status, Status::OK) << "setMode failed for mode " << (int)availableModes[0];
}

// E2E Test: Initialize -> SetMode -> Start -> Stop
TEST_F(E2ECameraWorkflowTest, InitializeToStartStop) {
    Status status = camera->initialize();
    ASSERT_EQ(status, Status::OK);
    
    std::vector<uint8_t> modes;
    status = camera->getAvailableModes(modes);
    ASSERT_EQ(status, Status::OK);
    ASSERT_FALSE(modes.empty());
    
    status = camera->setMode(modes[0]);
    ASSERT_EQ(status, Status::OK);
    
    // Start streaming
    status = camera->start();
    EXPECT_EQ(status, Status::OK) << "Camera start failed";
    
    // Stop streaming
    status = camera->stop();
    EXPECT_EQ(status, Status::OK) << "Camera stop failed";
}

// E2E Test: Complete workflow with single frame capture
TEST_F(E2ECameraWorkflowTest, CompleteWorkflowSingleFrame) {
    // Initialize
    Status status = camera->initialize();
    ASSERT_EQ(status, Status::OK);
    
    // Get and set mode
    std::vector<uint8_t> modes;
    status = camera->getAvailableModes(modes);
    ASSERT_EQ(status, Status::OK);
    ASSERT_FALSE(modes.empty());
    
    status = camera->setMode(modes[0]);
    ASSERT_EQ(status, Status::OK);
    
    // Start
    status = camera->start();
    ASSERT_EQ(status, Status::OK);
    
    // Request frame
    Frame frame;
    status = camera->requestFrame(&frame);
    EXPECT_EQ(status, Status::OK) << "requestFrame failed";
    
    // Verify frame has data
    if (status == Status::OK) {
        FrameDetails details;
        status = frame.getDetails(details);
        EXPECT_EQ(status, Status::OK);
    }
    
    // Stop
    status = camera->stop();
    EXPECT_EQ(status, Status::OK);
}

// E2E Test: Multiple frame capture workflow
TEST_F(E2ECameraWorkflowTest, CompleteWorkflowMultipleFrames) {
    Status status = camera->initialize();
    ASSERT_EQ(status, Status::OK);
    
    std::vector<uint8_t> modes;
    status = camera->getAvailableModes(modes);
    ASSERT_EQ(status, Status::OK);
    ASSERT_FALSE(modes.empty());
    
    status = camera->setMode(modes[0]);
    ASSERT_EQ(status, Status::OK);
    
    status = camera->start();
    ASSERT_EQ(status, Status::OK);
    
    // Capture multiple frames
    const int NUM_FRAMES = 5;
    int successfulFrames = 0;
    
    for (int i = 0; i < NUM_FRAMES; i++) {
        Frame frame;
        status = camera->requestFrame(&frame);
        if (status == Status::OK) {
            successfulFrames++;
        }
    }
    
    EXPECT_GT(successfulFrames, 0) << "No frames captured successfully";
    
    status = camera->stop();
    EXPECT_EQ(status, Status::OK);
    
    ::testing::Test::RecordProperty("FramesCaptured", successfulFrames);
    ::testing::Test::RecordProperty("FramesRequested", NUM_FRAMES);
}

// E2E Test: Mode switching workflow
TEST_F(E2ECameraWorkflowTest, ModeSwitchingWorkflow) {
    Status status = camera->initialize();
    ASSERT_EQ(status, Status::OK);
    
    std::vector<uint8_t> modes;
    status = camera->getAvailableModes(modes);
    ASSERT_EQ(status, Status::OK);
    
    if (modes.size() < 2) {
        GTEST_SKIP_("Need at least 2 modes for mode switching test");
    }
    
    // Test mode switching
    for (size_t i = 0; i < std::min(modes.size(), (size_t)3); i++) {
        status = camera->setMode(modes[i]);
        EXPECT_EQ(status, Status::OK) << "Failed to set mode " << (int)modes[i];
        
        // Verify mode was set by starting and capturing
        status = camera->start();
        if (status == Status::OK) {
            Frame frame;
            camera->requestFrame(&frame);
            camera->stop();
        }
    }
}

// E2E Test: Restart workflow (stop and restart streaming)
TEST_F(E2ECameraWorkflowTest, RestartStreamingWorkflow) {
    Status status = camera->initialize();
    ASSERT_EQ(status, Status::OK);
    
    std::vector<uint8_t> modes;
    status = camera->getAvailableModes(modes);
    ASSERT_EQ(status, Status::OK);
    ASSERT_FALSE(modes.empty());
    
    status = camera->setMode(modes[0]);
    ASSERT_EQ(status, Status::OK);
    
    // First streaming cycle
    status = camera->start();
    ASSERT_EQ(status, Status::OK);
    
    Frame frame1;
    status = camera->requestFrame(&frame1);
    EXPECT_EQ(status, Status::OK);
    
    status = camera->stop();
    ASSERT_EQ(status, Status::OK);
    
    // Second streaming cycle (restart)
    status = camera->start();
    EXPECT_EQ(status, Status::OK) << "Failed to restart streaming";
    
    Frame frame2;
    status = camera->requestFrame(&frame2);
    EXPECT_EQ(status, Status::OK) << "Failed to capture frame after restart";
    
    status = camera->stop();
    EXPECT_EQ(status, Status::OK);
}

int main(int argc, char** argv) {
    aditof_test::TestRunner runner(argv[0]);
    
    int initResult = runner.initialize(argc, argv);
    if (initResult != -1) {
        return initResult;
    }
    
    return runner.runTests();
}
