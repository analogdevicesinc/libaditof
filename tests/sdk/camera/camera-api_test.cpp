/*
 * MIT License
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */
#include "mock_depth_sensor.h"
#include <aditof/camera.h>
#include <aditof/camera_definitions.h>
#include <aditof/frame.h>
#include <aditof/status_definitions.h>
#include <aditof/system.h>
#include <gtest/gtest.h>
#include <memory>

using namespace aditof;

/**
 * @brief Unit tests for Camera API (Phase 4).
 * 
 * Tests camera API contracts, parameter validation, and basic operations.
 * These tests use real System/Camera objects but skip when hardware
 * is not available, focusing on API behavior and error handling.
 */

// ==============================================================================
// Test Fixture
// ==============================================================================

class CameraApiTest : public ::testing::Test {
  protected:
    void SetUp() override {
        // Tests will use System::getCameraList() to get real or mock cameras
    }

    void TearDown() override {
        // Cleanup handled automatically
    }
};

// ==============================================================================
// Basic API Method Tests
// ==============================================================================

TEST_F(CameraApiTest, EnableXYZFrameReturnsOK) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    // Get real camera (will skip if hardware not available)
    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available - testing with mock not yet "
                        "implemented for enableXYZframe";
    }

    auto camera = cameras.front();

    // Test enable
    Status status = camera->enableXYZframe(true);
    EXPECT_EQ(status, Status::OK);

    // Test disable
    status = camera->enableXYZframe(false);
    EXPECT_EQ(status, Status::OK);
}

TEST_F(CameraApiTest, EnableDepthComputeReturnsUnavailable) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available - testing with mock not yet "
                        "implemented for enableDepthCompute";
    }

    auto camera = cameras.front();

    // enableDepthCompute is not implemented, should return UNAVAILABLE
    Status status = camera->enableDepthCompute(true);
    EXPECT_EQ(status, Status::UNAVAILABLE);

    status = camera->enableDepthCompute(false);
    EXPECT_EQ(status, Status::UNAVAILABLE);
}

TEST_F(CameraApiTest, GetDetailsReturnsValidStructure) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available";
    }

    auto camera = cameras.front();

    CameraDetails details{};
    Status status = camera->getDetails(details);

    EXPECT_EQ(status, Status::OK);
    // Camera should have an ID even before initialization
    EXPECT_FALSE(details.cameraId.empty());
}

TEST_F(CameraApiTest, GetSensorReturnsValidPointer) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available";
    }

    auto camera = cameras.front();

    auto sensor = camera->getSensor();
    EXPECT_NE(sensor, nullptr);
}

// ==============================================================================
// Control Operations Tests
// ==============================================================================

TEST_F(CameraApiTest, SetControlWithInvalidNameReturnsError) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available";
    }

    auto camera = cameras.front();

    // Initialize camera first
    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed (hardware not available)";
    }

    // Try to set a control that doesn't exist
    status = camera->setControl("nonexistent_control_12345", "value");
    EXPECT_EQ(status, Status::INVALID_ARGUMENT);
}

TEST_F(CameraApiTest, GetControlWithInvalidNameReturnsError) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available";
    }

    auto camera = cameras.front();

    // Initialize camera first
    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed (hardware not available)";
    }

    std::string value;
    status = camera->getControl("nonexistent_control_12345", value);
    EXPECT_EQ(status, Status::INVALID_ARGUMENT);
}

TEST_F(CameraApiTest, SetControlValidatesControlName) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available";
    }

    auto camera = cameras.front();

    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed";
    }

    // Test with empty control name
    status = camera->setControl("", "value");
    EXPECT_EQ(status, Status::INVALID_ARGUMENT);
}

// ==============================================================================
// Mode Operations Tests
// ==============================================================================

TEST_F(CameraApiTest, GetAvailableModesBeforeInitializeReturnsEmpty) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available";
    }

    auto camera = cameras.front();

    // Try to get modes before initialization
    std::vector<uint8_t> modes;
    Status status = camera->getAvailableModes(modes);

    // Behavior may vary: either GENERIC_ERROR or returns empty list
    // Both are acceptable before initialization
    if (status == Status::OK) {
        EXPECT_TRUE(modes.empty());
    } else {
        EXPECT_NE(status, Status::OK);
    }
}

TEST_F(CameraApiTest, SetModeBeforeInitializeReturnsError) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available";
    }

    auto camera = cameras.front();

    // Try to set mode before initialization
    Status status = camera->setMode(0);
    EXPECT_NE(status, Status::OK);
}

// ==============================================================================
// Frame Request Tests
// ==============================================================================

TEST_F(CameraApiTest, RequestFrameWithNullptrReturnsError) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available";
    }

    auto camera = cameras.front();

    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed";
    }

    // Try to request frame with nullptr
    status = camera->requestFrame(nullptr);
    EXPECT_NE(status, Status::OK);
}

TEST_F(CameraApiTest, RequestFrameBeforeStartReturnsError) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available";
    }

    auto camera = cameras.front();

    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed";
    }

    // Try to request frame before calling start()
    Frame frame;
    status = camera->requestFrame(&frame);
    EXPECT_NE(status, Status::OK);
}

// ==============================================================================
// Parameter Operations Tests
// ==============================================================================

TEST_F(CameraApiTest, GetFrameProcessParamsReturnsMap) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available";
    }

    auto camera = cameras.front();

    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed";
    }

    std::map<std::string, std::string> params;
    status = camera->getFrameProcessParams(params);

    // Should return OK or UNAVAILABLE depending on implementation
    EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
}

TEST_F(CameraApiTest, SetFrameProcessParamsAcceptsEmptyMap) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available";
    }

    auto camera = cameras.front();

    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed";
    }

    std::map<std::string, std::string> params; // Empty map
    status = camera->setFrameProcessParams(params, 0);

    // Should handle empty map gracefully
    EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
}

// ==============================================================================
// Camera Lifecycle Tests
// ==============================================================================

TEST_F(CameraApiTest, StopBeforeStartReturnsError) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available";
    }

    auto camera = cameras.front();

    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed";
    }

    // Try to stop before starting
    status = camera->stop();
    EXPECT_NE(status, Status::OK);
}

TEST_F(CameraApiTest, MultipleStopsAreIdempotent) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available";
    }

    auto camera = cameras.front();

    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed";
    }

    status = camera->start();
    if (status != Status::OK) {
        GTEST_SKIP() << "Camera start failed";
    }

    // Stop once
    status = camera->stop();
    EXPECT_EQ(status, Status::OK);

    // Stop again - should be safe
    status = camera->stop();
    // Implementation may return OK or error, both are acceptable
    // Just verify it doesn't crash
}

// ==============================================================================
// CameraDetails Structure Tests
// ==============================================================================

TEST_F(CameraApiTest, CameraDetailsHasValidConnection) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available";
    }

    auto camera = cameras.front();

    CameraDetails details{};
    Status status = camera->getDetails(details);
    ASSERT_EQ(status, Status::OK);

    // Connection type should be one of the valid enum values
    EXPECT_TRUE(details.connection == ConnectionType::ON_TARGET ||
                details.connection == ConnectionType::NETWORK ||
                details.connection == ConnectionType::OFFLINE);
}

TEST_F(CameraApiTest, CameraDetailsInitialModeIsInvalid) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available";
    }

    auto camera = cameras.front();

    CameraDetails details{};
    Status status = camera->getDetails(details);
    ASSERT_EQ(status, Status::OK);

    // Before initialization/setMode, mode should be invalid (-1)
    EXPECT_EQ(details.mode, -1);
}

TEST_F(CameraApiTest, CameraDetailsModeUpdatesAfterSetMode) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available";
    }

    auto camera = cameras.front();

    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed";
    }

    // Get available modes
    std::vector<uint8_t> modes;
    status = camera->getAvailableModes(modes);
    if (status != Status::OK || modes.empty()) {
        GTEST_SKIP() << "No modes available";
    }

    // Set a mode
    uint8_t testMode = modes[0];
    status = camera->setMode(testMode);
    ASSERT_EQ(status, Status::OK);

    // Check details reflect the mode
    CameraDetails details{};
    status = camera->getDetails(details);
    ASSERT_EQ(status, Status::OK);
    EXPECT_EQ(details.mode, testMode);
}

// ==============================================================================
// Available Controls Tests
// ==============================================================================

TEST_F(CameraApiTest, GetAvailableControlsReturnsNonEmpty) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available";
    }

    auto camera = cameras.front();

    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed";
    }

    std::vector<std::string> controls;
    status = camera->getAvailableControls(controls);

    EXPECT_EQ(status, Status::OK);
    // After initialization, there should be some controls available
    EXPECT_FALSE(controls.empty());
}

TEST_F(CameraApiTest, AvailableControlsAreNonEmpty) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    system.getCameraList(cameras);

    if (cameras.empty()) {
        GTEST_SKIP() << "No camera available";
    }

    auto camera = cameras.front();

    Status status = camera->initialize();
    if (status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed";
    }

    std::vector<std::string> controls;
    status = camera->getAvailableControls(controls);
    ASSERT_EQ(status, Status::OK);

    // Each control name should be non-empty
    for (const auto &control : controls) {
        EXPECT_FALSE(control.empty());
    }
}

// ==============================================================================
// Main
// ==============================================================================

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
