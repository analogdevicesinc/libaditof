#include <gtest/gtest.h>
#include <aditof/version.h>
#include <aditof/system.h>
#include <aditof_test_utils.h>
#include <iostream>
#include <vector>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/camera_definitions.h>
#include <aditof/depth_sensor_interface.h>
#include <aditof/frame.h>
#include <aditof/status_definitions.h>
#include <aditof/depth_sensor_interface.h>
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

TEST(SystemTest, GetCameraListWithoutCameras) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;

    // This should not throw even if no cameras are connected
    EXPECT_NO_THROW({
        if (g_cameraipaddress == "") {
            system.getCameraList(cameras);
        } else {
            system.getCameraList(cameras, "ip:" + g_cameraipaddress);
        }
    });

    // The cameras vector might be empty if no hardware is connected
    // This is expected in a test environment
}

bool g_callbackInvoked = false;
// Test fixture for Camera-related tests
class CameraTestFixture : public ::testing::Test {
  protected:
    void SetUp() override {
        system = std::make_unique<System>();

        // Try to get cameras, but don't fail if none are available
        if (g_cameraipaddress == "") {
            system->getCameraList(cameras);
        } else {
            system->getCameraList(cameras, "ip:" + g_cameraipaddress);
        }

        if (!cameras.empty()) {
            camera = cameras.front();
            has_camera = true;

            g_callbackInvoked = false;
            // Registering a callback to be executed when ADSD3500 issues an interrupt
            std::shared_ptr<DepthSensorInterface> sensor = camera->getSensor();
            callback = [this](Adsd3500Status status) {
                g_callbackInvoked = true;
                EXPECT_EQ(status, Adsd3500Status::OK);
            };

            Status registerCbStatus =
                sensor->adsd3500_register_interrupt_callback(callback);
            ASSERT_TRUE(registerCbStatus == Status::OK);
        } else {
            has_camera = false;
        }
    }

    void TearDown() override {
        camera = cameras.front();
        std::shared_ptr<DepthSensorInterface> sensor = camera->getSensor();
        sensor->adsd3500_unregister_interrupt_callback(callback);
        // No explicit cleanup needed - cameras will be cleaned up automatically
        cameras.clear();
        camera.reset();
    }

    std::unique_ptr<System> system;
    std::vector<std::shared_ptr<Camera>> cameras;
    std::shared_ptr<Camera> camera;
    bool has_camera = false;
    aditof::SensorInterruptCallback callback;
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

TEST_F(CameraTestFixture, adsd3500SoftReset) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    Status init_status = camera->initialize();

    if (init_status == Status::OK) {

        // Reset the flag before the test operations
        ASSERT_TRUE(g_callbackInvoked == true);
        g_callbackInvoked = false;

        Status status = camera->adsd3500SetFrameRate(23);
        ASSERT_TRUE(status == Status::OK);

        uint16_t frameRate = 0;
        status = camera->adsd3500GetFrameRate(frameRate);
        ASSERT_TRUE(status == Status::OK);
        ASSERT_TRUE(frameRate == 23);

        status = camera->adsd3500SetGenericTemplate(0x0024, 0);
        ASSERT_TRUE(status == Status::OK);

        //Sleep for 5 seconds to allow time for the reset to occur
        std::this_thread::sleep_for(std::chrono::seconds(5));

        frameRate = 0;
        status = camera->adsd3500GetFrameRate(frameRate);
        ASSERT_TRUE(status == Status::OK);
        ASSERT_FALSE(frameRate == 23);
        ASSERT_TRUE(frameRate == 10);
    } else {
        GTEST_SKIP() << "Camera initialization failed, skipping test";
    }
}

int main(int argc, char **argv) {
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