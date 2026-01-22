#include <aditof/camera.h>
#include <aditof/camera_definitions.h>
#include <aditof/frame.h>
#include <aditof/status_definitions.h>
#include <aditof/system.h>
#include <aditof/version.h>
#include <gtest/gtest.h>

using namespace aditof;

// Test version information
TEST(VersionTest, VersionMacrosAreDefined) {
    // Version macros are strings, just verify they're not empty
    std::string major(ADITOF_API_VERSION_MAJOR);
    std::string minor(ADITOF_API_VERSION_MINOR);
    std::string patch(ADITOF_API_VERSION_PATCH);

    EXPECT_FALSE(major.empty());
    EXPECT_FALSE(minor.empty());
    EXPECT_FALSE(patch.empty());
}

TEST(VersionTest, ApiVersionReturnsNonEmptyString) {
    std::string version = getApiVersion();
    EXPECT_FALSE(version.empty());
    // Version should contain dots
    EXPECT_NE(version.find('.'), std::string::npos);
}

// Test System class
TEST(SystemTest, SystemInstantiation) {
    EXPECT_NO_THROW({ System system; });
}

TEST(SystemTest, GetCameraListWithoutCameras) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    // This should not throw even if no cameras are connected
    EXPECT_NO_THROW({ system.getCameraList(cameras); });

    // The cameras vector might be empty if no hardware is connected
    // This is expected in a test environment
}

// Test Frame class
TEST(FrameTest, FrameInstantiation) {
    EXPECT_NO_THROW({ Frame frame; });
}

TEST(FrameTest, FrameDetailsAccess) {
    Frame frame;
    FrameDetails details;

    Status status = frame.getDetails(details);
    // Frame might not be initialized, but the call should not crash
    EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
}

// Test fixture for Camera-related tests
class CameraTestFixture : public ::testing::Test {
  protected:
    void SetUp() override {
        system = std::make_unique<System>();

        // Try to get cameras, but don't fail if none are available
        system->getCameraList(cameras);

        if (!cameras.empty()) {
            camera = cameras.front();
            has_camera = true;
        } else {
            has_camera = false;
        }
    }

    void TearDown() override {
        // No explicit cleanup needed - cameras will be cleaned up automatically
        cameras.clear();
        camera.reset();
    }

    std::unique_ptr<System> system;
    std::vector<std::shared_ptr<Camera>> cameras;
    std::shared_ptr<Camera> camera;
    bool has_camera = false;
};

TEST_F(CameraTestFixture, SystemHasCameraListMethod) {
    EXPECT_NE(system, nullptr);
    // This test passes if we can call getCameraList without crashing
}

TEST_F(CameraTestFixture, CameraDetailsAccessible) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    CameraDetails details;
    Status status = camera->getDetails(details);

    // If we have a camera, we should be able to get its details
    if (status == Status::OK) {
        EXPECT_FALSE(details.cameraId.empty());
    }
}

TEST_F(CameraTestFixture, CameraAvailableModesQuery) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    // Initialize camera first (might fail in test environment)
    Status init_status = camera->initialize();

    if (init_status == Status::OK) {
        std::vector<uint8_t> modes;
        Status status = camera->getAvailableModes(modes);
        EXPECT_EQ(status, Status::OK);
    } else {
        GTEST_SKIP() << "Camera initialization failed, skipping test";
    }
}

// Test Status enum
TEST(StatusTest, StatusEnumValues) {
    EXPECT_EQ(static_cast<int>(Status::OK), 0);
    EXPECT_NE(static_cast<int>(Status::GENERIC_ERROR), 0);
    EXPECT_NE(static_cast<int>(Status::INVALID_ARGUMENT), 0);
}

// Parameterized test for frame types
class FrameTypeTest : public ::testing::TestWithParam<std::string> {};

TEST_P(FrameTypeTest, FrameTypeStringValid) {
    std::string frameType = GetParam();
    EXPECT_FALSE(frameType.empty());
    // Common frame types should have reasonable length
    EXPECT_LT(frameType.length(), 20);
}

INSTANTIATE_TEST_SUITE_P(CommonFrameTypes, FrameTypeTest,
                         ::testing::Values("depth", "ir", "ab", "xyz", "conf"));
