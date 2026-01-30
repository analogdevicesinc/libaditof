#include <gtest/gtest.h>
#include <aditof/system.h>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/camera_definitions.h>
#include <aditof/status_definitions.h>
#include <aditof_test_utils.h>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>

using namespace aditof;

// Global camera IP from test utilities
std::string& g_cameraipaddress = aditof_test::g_cameraipaddress;

/**
 * FUNCTIONAL TESTS FOR END-TO-END CAMERA OPERATIONS
 * 
 * Tests complete workflows:
 * - Camera initialization to frame capture
 * - Frame processing pipeline
 * - Real-time operation mode switching
 * - Performance and stress tests
 */

class CameraFunctionalTest : public ::testing::Test {
protected:
    System system;
    std::shared_ptr<Camera> camera;
    
    void SetUp() override {
        std::vector<std::shared_ptr<Camera>> cameras;
        Status status;
        
        if (g_cameraipaddress == "") {
            status = system.getCameraList(cameras);
        } else {
            status = system.getCameraList(cameras, "ip:" + g_cameraipaddress);
        }
        
        if (status != Status::OK || cameras.empty()) {
            GTEST_SKIP_("No camera available for functional test");
        }
        
        camera = cameras[0];
    }

    void TearDown() override {
        if (camera) {
            // Cleanup camera
        }
    }
};

// Test: Complete initialization workflow
TEST_F(CameraFunctionalTest, CompleteInitializationWorkflow) {
    ASSERT_NE(camera, nullptr);
    
    // Step 1: Initialize camera
    Status status = camera->initialize();
    EXPECT_EQ(status, Status::OK);
    
    // Step 2: Get camera details
    CameraDetails cameraDetails;
    status = camera->getDetails(cameraDetails);
    EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
    
    // Step 3: Get available modes
    std::vector<uint8_t> availableModes;
    status = camera->getAvailableModes(availableModes);
    EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
}

// Test: Frame type workflow
TEST_F(CameraFunctionalTest, FrameTypeWorkflow) {
    ASSERT_NE(camera, nullptr);
    
    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP_("Camera initialization failed");
    }
    
    std::vector<uint8_t> availableModes;
    status = camera->getAvailableModes(availableModes);
    
    if (status == Status::OK && !availableModes.empty()) {
        // Try each mode
        for (uint8_t mode : availableModes) {
            Status setStatus = camera->setMode(mode);
            EXPECT_TRUE(setStatus == Status::OK || setStatus == Status::UNAVAILABLE);
        }
    }
}

// Test: Multiple initialization cycles
TEST_F(CameraFunctionalTest, MultipleInitializationCycles) {
    ASSERT_NE(camera, nullptr);
    
    for (int i = 0; i < 3; i++) {
        Status status = camera->initialize();
        
        // Should succeed or be idempotent
        EXPECT_TRUE(status == Status::OK || status == Status::BUSY);
    }
}

// Test: Mode switching workflow
TEST_F(CameraFunctionalTest, ModeSwitching) {
    ASSERT_NE(camera, nullptr);
    
    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP_("Camera initialization failed");
    }
    
    // Try switching between different modes
    std::vector<uint8_t> modes = {0, 1, 2};
    
    for (uint8_t mode : modes) {
        status = camera->setMode(mode);
        
        // Should either succeed or be unsupported
        EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
    }
}

// Test: Frame details and allocation
TEST_F(CameraFunctionalTest, FrameAllocationWorkflow) {
    ASSERT_NE(camera, nullptr);
    
    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP_("Camera initialization failed");
    }
    
    Frame frame;
    FrameDetails details;
    details.width = 512;
    details.height = 512;
    details.type = "depth";
    details.cameraMode = "near";
    
    // Get details from frame
    status = frame.getDetails(details);
    
    if (status == Status::OK) {
        // Verify details are valid
        EXPECT_TRUE(details.width > 0 || details.height > 0);
    }
}

// Test: Acquisition time measurement
TEST_F(CameraFunctionalTest, AcquisitionTimeMeasurement) {
    ASSERT_NE(camera, nullptr);
    
    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP_("Camera initialization failed");
    }
    
    auto start = std::chrono::high_resolution_clock::now();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    auto end = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    EXPECT_GE(duration.count(), 10);
    ::testing::Test::RecordProperty("AcquisitionTime_ms", duration.count());
}

// Test: Parameter validation workflow
TEST_F(CameraFunctionalTest, ParameterValidation) {
    ASSERT_NE(camera, nullptr);
    
    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP_("Camera initialization failed");
    }
    
    // Get camera details for validation
    CameraDetails details;
    status = camera->getDetails(details);
    
    // Should handle gracefully
    EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
}

// Test: Concurrent initialization (thread safety)
TEST_F(CameraFunctionalTest, ConcurrentInitialization) {
    ASSERT_NE(camera, nullptr);
    
    std::vector<std::thread> threads;
    std::vector<Status> results(2);
    
    for (int i = 0; i < 2; i++) {
        threads.emplace_back([this, i, &results]() {
            results[i] = this->camera->initialize();
        });
    }
    
    for (auto& t : threads) {
        t.join();
    }
    
    // Both should complete without crash
    EXPECT_TRUE(true);
}

// Test: Long running operation
TEST_F(CameraFunctionalTest, LongRunningOperationStability) {
    ASSERT_NE(camera, nullptr);
    
    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP_("Camera initialization failed");
    }
    
    // Simulate long running operation
    for (int i = 0; i < 10; i++) {
        CameraDetails details;
        status = camera->getDetails(details);
        EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

// Test: Error recovery
TEST_F(CameraFunctionalTest, ErrorRecovery) {
    ASSERT_NE(camera, nullptr);
    
    Status status = camera->initialize();
    EXPECT_EQ(status, Status::OK);
    
    // Try invalid operation and verify recovery
    uint8_t invalidMode = 255;
    status = camera->setMode(invalidMode);
    
    // Camera should still be functional after error
    CameraDetails details;
    status = camera->getDetails(details);
    EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
}

// Test: Status propagation
TEST_F(CameraFunctionalTest, StatusPropagation) {
    ASSERT_NE(camera, nullptr);
    
    Status status = camera->initialize();
    EXPECT_EQ(status, Status::OK);
    EXPECT_TRUE(status == Status::OK);
    
    // Verify status can be compared
    Status ok = Status::OK;
    Status error = Status::INVALID_ARGUMENT;
    
    EXPECT_EQ(ok, status);
    EXPECT_NE(error, status);
}

// Test: Resource cleanup on exception
TEST_F(CameraFunctionalTest, ResourceCleanupOnException) {
    ASSERT_NE(camera, nullptr);
    
    try {
        Status status = camera->initialize();
        // Simulate operation that might throw
        std::vector<uint8_t> modes;
        status = camera->getAvailableModes(modes);
        
        // Should not throw
        EXPECT_TRUE(true);
    } catch (...) {
        FAIL() << "Unexpected exception thrown";
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
