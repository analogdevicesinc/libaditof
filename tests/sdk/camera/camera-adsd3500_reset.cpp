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

            // Cast to hardware-specific interface for ADSD3500 operations
            hardwareSensor =
                std::dynamic_pointer_cast<Adsd3500HardwareInterface>(sensor);

            if (hardwareSensor) {
                callback = [this](Adsd3500Status status) {
                    g_callbackInvoked = true;
                    EXPECT_EQ(status, Adsd3500Status::OK);
                };

                Status registerCbStatus =
                    hardwareSensor->adsd3500_register_interrupt_callback(
                        callback);
                // Don't fail if callback registration not supported - just log it
                if (registerCbStatus != Status::OK) {
                    std::cout
                        << "Note: Interrupt callback registration not available"
                        << std::endl;
                }
            }
        } else {
            has_camera = false;
        }
    }

    void TearDown() override {
        if (hardwareSensor) {
            hardwareSensor->adsd3500_unregister_interrupt_callback(callback);
        }
        // No explicit cleanup needed - cameras will be cleaned up automatically
        cameras.clear();
        camera.reset();
    }

    std::unique_ptr<System> system;
    std::vector<std::shared_ptr<Camera>> cameras;
    std::shared_ptr<Camera> camera;
    std::shared_ptr<Adsd3500HardwareInterface> hardwareSensor;
    bool has_camera = false;
    aditof::SensorInterruptCallback callback;
};

TEST_F(CameraTestFixture, adsd3500SoftReset) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    if (!hardwareSensor) {
        GTEST_SKIP() << "Hardware sensor interface not available";
    }

    Status init_status = camera->initialize();

    if (init_status == Status::OK) {

        // Reset the callback flag before testing
        g_callbackInvoked = false;

        // Perform soft reset using proper SDK method
        aditof::Status status;
        status = hardwareSensor->adsd3500_reset();
        ASSERT_TRUE(status == Status::OK);

        // Sleep to allow time for reset to complete and camera to reinitialize
        std::this_thread::sleep_for(std::chrono::seconds(3));

        // Read chip ID after reset to verify chip is responsive
        uint16_t chipId = 0;
        status = hardwareSensor->adsd3500_read_cmd(0x0112, &chipId);
        ASSERT_TRUE(status == Status::OK)
            << "Failed to read chip ID after reset";
        ASSERT_EQ(chipId, 0x5931)
            << "Chip ID should be 0x5931 after reset, got: 0x" << std::hex
            << chipId << std::dec;
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