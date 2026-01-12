#include <gtest/gtest.h>
#include <aditof/version.h>
#include <aditof/system.h>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <vector>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/camera_definitions.h>
#include <aditof/status_definitions.h>
#include <aditof/depth_sensor_interface.h>
#include <array>
#include <cmath>
#include <thread>

using namespace aditof;

std::string g_cameraipaddress = "";

// Generate UTC timestamp in format: YYYYMMDD_HHMMSS
std::string getUTCTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    
    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&time_t_now), "%Y%m%d_%H%M%S");
    return oss.str();
}

// Test System class
TEST(SystemTest, SystemInstantiation) {
    EXPECT_NO_THROW({
        System system;
    });
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

TEST_F(CameraTestFixture, adsd3500Reset) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    Status init_status = camera->initialize();
    
    if (init_status == Status::OK) {
    
        // Registering a callback to be executed when ADSD3500 issues an interrupt
        std::shared_ptr<DepthSensorInterface> sensor = camera->getSensor();
        aditof::SensorInterruptCallback callback = [](Adsd3500Status status) {
            std::cout << "ADSD3500 Interrupt Callback invoked with status: "
                    << static_cast<int>(status) << std::endl;
        };
        
        Status registerCbStatus =
            sensor->adsd3500_register_interrupt_callback(callback);
        ASSERT_TRUE(registerCbStatus == Status::OK);

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

int main(int argc, char** argv) {

    bool bHelp = false;
    // Parse custom command-line argument for expected version
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg.find("--ip=") == 0) {
            g_cameraipaddress = arg.substr(5);  // Extract IP address after "--ip="
        }  else if (arg == "--help" || arg == "-h") {
            bHelp = true;
        }
    }


    // Automatically add --gtest_output with timestamped filename
    std::string timestamp = getUTCTimestamp();
    std::string execName = argv[0];
    // Extract just the executable name without path
    size_t lastSlash = execName.find_last_of("/\\");
    if (lastSlash != std::string::npos) {
        execName = execName.substr(lastSlash + 1);
    }
    std::string gtestOutput = "--gtest_output=json:report_" + execName + "_" + timestamp + ".json";
    
    // Create new argv with the additional argument
    std::vector<char*> newArgv;
    for (int i = 0; i < argc; ++i) {
        newArgv.push_back(argv[i]);
    }
    newArgv.push_back(const_cast<char*>(gtestOutput.c_str()));
    newArgv.push_back(nullptr);
    
    int newArgc = argc + 1;

    if (bHelp) {
        std::cout << "Usage: " << argv[0] << " [--ip=<camera_ip_address>] [--help|-h]" << std::endl;
        std::cout << "  --ip: Specify the camera IP address" << std::endl;
        std::cout << std::endl;
    }
    ::testing::InitGoogleTest(&newArgc, newArgv.data());

    return RUN_ALL_TESTS();
}