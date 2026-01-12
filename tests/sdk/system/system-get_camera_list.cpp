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

TEST(SystemTest, GetCameraListCameras) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    Status status;
    // This should not throw even if no cameras are connected
    EXPECT_NO_THROW({
        if (g_cameraipaddress == "") {
            status = system.getCameraList(cameras);
        } else {
            status = system.getCameraList(cameras, "ip:" + g_cameraipaddress);
        }
    });

    ASSERT_TRUE(status == Status::OK);
   
    bool has_camera = false;
    if (!cameras.empty()) {
        auto camera = cameras.front();
        has_camera = true;
    } else {
        has_camera = false;
    }

    ASSERT_TRUE(has_camera);
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