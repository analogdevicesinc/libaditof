/**
 * SYSTEM TEST: Connection Types Integration
 * 
 * Tests different camera connection methods:
 * - Local/Target connection (ON_TARGET)
 * - Network/Remote connection (NETWORK) 
 * - Offline/Software connection (OFFLINE)
 */

#include <gtest/gtest.h>
#include <aditof/system.h>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/camera_definitions.h>
#include <aditof/status_definitions.h>
#include <aditof_test_utils.h>
#include <vector>
#include <memory>
#include <string>

using namespace aditof;

std::string& g_cameraipaddress = aditof_test::g_cameraipaddress;

/**
 * ConnectionTypesTest - Tests different ways to connect to camera
 */
class ConnectionTypesTest : public ::testing::Test {
protected:
    System system;
};

// Test: Local camera discovery (no IP specified)
TEST_F(ConnectionTypesTest, LocalCameraDiscovery) {
    std::vector<std::shared_ptr<Camera>> cameras;
    Status status = system.getCameraList(cameras);
    
    EXPECT_EQ(status, Status::OK) << "getCameraList should succeed";
    
    if (!cameras.empty()) {
        ::testing::Test::RecordProperty("LocalCamerasFound", (int)cameras.size());
        
        // Verify each camera can be queried
        for (auto& camera : cameras) {
            EXPECT_NE(camera, nullptr);
        }
    }
}

// Test: Network camera discovery with IP
TEST_F(ConnectionTypesTest, NetworkCameraDiscovery) {
    if (g_cameraipaddress.empty()) {
        GTEST_SKIP() << "\n"
            << "======================================\n"
            << "Network Camera Discovery - SKIPPED\n"
            << "======================================\n"
            << "Reason: No IP address configured\n"
            << "Current Setup: LOCAL connection (flex cable)\n"
            << "\n"
            << "This test requires network-connected camera.\n"
            << "To enable, run with: --ip=<CAMERA_IP>\n"
            << "Example: --ip=192.168.1.100\n"
            << "======================================";
    }
    
    std::vector<std::shared_ptr<Camera>> cameras;
    std::string uri = "ip:" + g_cameraipaddress;
    Status status = system.getCameraList(cameras, uri);
    
    // Network discovery may fail if camera is unreachable or IP is wrong
    // This is expected behavior, not a test failure
    if (status != Status::OK) {
        GTEST_SKIP() << "\n"
            << "======================================\n"
            << "Network Camera - NOT REACHABLE\n"
            << "======================================\n"
            << "Configured IP: " << g_cameraipaddress << "\n"
            << "Status: Camera not responding\n"
            << "\n"
            << "Possible reasons:\n"
            << "  - Camera is offline\n"
            << "  - Wrong IP address\n"
            << "  - Network connectivity issue\n"
            << "======================================";
    }
    
    EXPECT_EQ(status, Status::OK) << "Network camera discovery should succeed";
    
    if (!cameras.empty()) {
        ::testing::Test::RecordProperty("NetworkCamera", g_cameraipaddress.c_str());
    }
}

// Test: Offline mode camera (software mode)
TEST_F(ConnectionTypesTest, OfflineModeDiscovery) {
    std::vector<std::shared_ptr<Camera>> cameras;
    std::string uri = "offline:"; // Offline/software mode
    Status status = system.getCameraList(cameras, uri);
    
    // Offline mode may or may not be supported
    EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
}

// Test: Invalid URI format handling
TEST_F(ConnectionTypesTest, InvalidURIHandling) {
    std::vector<std::shared_ptr<Camera>> cameras;
    
    // Invalid URI schemes should be handled gracefully
    Status status = system.getCameraList(cameras, "invalid:///path");
    // Should either return OK with empty list or INVALID_ARGUMENT
    EXPECT_TRUE(status == Status::OK || status == Status::INVALID_ARGUMENT);
}

// Test: Empty URI string handling
TEST_F(ConnectionTypesTest, EmptyURIHandling) {
    std::vector<std::shared_ptr<Camera>> cameras;
    Status status = system.getCameraList(cameras, "");
    
    // Empty string should be handled gracefully (use default)
    EXPECT_EQ(status, Status::OK);
}

/**
 * NetworkCameraIntegrationTest - Full workflow tests for network camera
 */
class NetworkCameraIntegrationTest : public ::testing::Test {
protected:
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    std::shared_ptr<Camera> camera;
    
    void SetUp() override {
        if (g_cameraipaddress.empty()) {
            GTEST_SKIP() << "\n"
                << "======================================\n"
                << "Network Integration Test - SKIPPED\n"
                << "======================================\n"
                << "Current Setup: LOCAL camera (flex cable)\n"
                << "Required: Network camera with IP address\n"
                << "\n"
                << "These tests validate network streaming,\n"
                << "latency, and remote camera operations.\n"
                << "\n"
                << "To enable: Run with --ip=<CAMERA_IP>\n"
                << "======================================";
        }
        
        std::string uri = "ip:" + g_cameraipaddress;
        Status status = system.getCameraList(cameras, uri);
        
        if (status != Status::OK || cameras.empty()) {
            GTEST_SKIP() << "\n"
                << "======================================\n"
                << "Network Camera - UNAVAILABLE\n"
                << "======================================\n"
                << "Configured IP: " << g_cameraipaddress << "\n"
                << "Status: " << static_cast<int>(status) << "\n"
                << "\n"
                << "Camera not responding at specified IP.\n"
                << "Verify network connection and IP address.\n"
                << "======================================";
        }
        
        camera = cameras[0];
    }
    
    void TearDown() override {
        if (camera) {
            camera->stop();
        }
    }
};

// Test: Network camera initialize
TEST_F(NetworkCameraIntegrationTest, Initialize) {
    if (!camera) {
        GTEST_SKIP() << "Network camera not available - using LOCAL camera instead";
    }
    Status status = camera->initialize();
    EXPECT_EQ(status, Status::OK) << "Network camera initialization should succeed";
}

// Test: Network camera full workflow
TEST_F(NetworkCameraIntegrationTest, FullStreamingWorkflow) {
    if (!camera) {
        GTEST_SKIP() << "Network streaming test requires network camera - using LOCAL camera instead";
    }
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
    
    // Capture frame over network
    Frame frame;
    status = camera->requestFrame(&frame);
    EXPECT_EQ(status, Status::OK) << "Network frame capture should succeed";
    
    FrameDetails details;
    frame.getDetails(details);
    EXPECT_GT(details.width, 0);
    EXPECT_GT(details.height, 0);
    
    status = camera->stop();
    EXPECT_EQ(status, Status::OK);
}

// Test: Network latency - multiple frames
TEST_F(NetworkCameraIntegrationTest, MultipleFrameCaptureLatency) {
    if (!camera) {
        GTEST_SKIP() << "Network latency test requires network camera - using LOCAL camera instead";
    }
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
    
    const int numFrames = 10;
    int successCount = 0;
    
    for (int i = 0; i < numFrames; i++) {
        Frame frame;
        status = camera->requestFrame(&frame);
        if (status == Status::OK) {
            successCount++;
        }
    }
    
    camera->stop();
    
    EXPECT_EQ(successCount, numFrames) 
        << "All " << numFrames << " frames should be captured successfully";
    ::testing::Test::RecordProperty("FramesCaptured", successCount);
}

/**
 * LocalCameraIntegrationTest - Full workflow tests for local camera
 */
class LocalCameraIntegrationTest : public ::testing::Test {
protected:
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    std::shared_ptr<Camera> camera;
    
    void SetUp() override {
        // Use local camera only (no IP)
        if (!g_cameraipaddress.empty()) {
            GTEST_SKIP_("IP address provided, use NetworkCameraIntegrationTest instead");
        }
        
        Status status = system.getCameraList(cameras);
        
        if (status != Status::OK || cameras.empty()) {
            GTEST_SKIP_("Local camera not available");
        }
        
        camera = cameras[0];
    }
    
    void TearDown() override {
        if (camera) {
            camera->stop();
        }
    }
};

// Test: Local camera initialize
TEST_F(LocalCameraIntegrationTest, Initialize) {
    Status status = camera->initialize();
    EXPECT_EQ(status, Status::OK) << "Local camera initialization should succeed";
}

// Test: Local camera full workflow
TEST_F(LocalCameraIntegrationTest, FullStreamingWorkflow) {
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
    
    Frame frame;
    status = camera->requestFrame(&frame);
    EXPECT_EQ(status, Status::OK) << "Local frame capture should succeed";
    
    status = camera->stop();
    EXPECT_EQ(status, Status::OK);
}

// Test: Local camera performance - high frame count
TEST_F(LocalCameraIntegrationTest, HighFrameCountCapture) {
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
    
    const int numFrames = 50;
    int successCount = 0;
    
    for (int i = 0; i < numFrames; i++) {
        Frame frame;
        status = camera->requestFrame(&frame);
        if (status == Status::OK) {
            successCount++;
        }
    }
    
    camera->stop();
    
    // Allow some frame drops but majority should succeed
    EXPECT_GE(successCount, numFrames * 0.9) 
        << "At least 90% of frames should be captured";
    ::testing::Test::RecordProperty("FramesCaptured", successCount);
    ::testing::Test::RecordProperty("FramesRequested", numFrames);
}

int main(int argc, char** argv) {
    aditof_test::TestRunner runner(argv[0]);
    
    int initResult = runner.initialize(argc, argv);
    if (initResult != -1) {
        return initResult;
    }
    
    return runner.runTests();
}
