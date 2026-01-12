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
#include <array>
#include <map>
#include <cmath>

using namespace aditof;

std::string g_cameraipaddress = "";

struct ModeDetails_struct {
    uint16_t width;
    uint16_t height;
};

const std::map<std::string, std::array<ModeDetails_struct, 10>> g_modeDetailsMap = {
    {"crosby", {{
        {1024, 1024}, // Mode 0
        {1024, 1024}, // Mode 1
        {512, 512},   // Mode 2
        {512, 512},   // Mode 3
        {0, 0},       // Mode 4 (invalid)
        {512, 512},   // Mode 5
        {512, 512},   // Mode 6
        {0, 0},       // Mode 7 (invalid)
        {0, 0},       // Mode 8 (invalid)
        {0, 0}        // Mode 9 (invalid)
    }}},
    {"tembinv2", {{
        {640, 512},   // Mode 0
        {640, 512},   // Mode 1
        {320, 240},   // Mode 2
        {320, 240},   // Mode 3
        {0, 0},       // Mode 4 (invalid)
        {320, 240},   // Mode 5
        {320, 240},   // Mode 6
        {0, 0},       // Mode 7 (invalid)
        {0, 0},       // Mode 8 (invalid)
        {0, 0}        // Mode 9 (invalid)
    }}},
    {"mystic", {{
        {640, 512},   // Mode 0
        {640, 512},   // Mode 1
        {320, 240},   // Mode 2
        {320, 240},   // Mode 3
        {0, 0},       // Mode 4 (invalid)
        {320, 240},   // Mode 5
        {320, 240},   // Mode 6
        {0, 0},       // Mode 7 (invalid)
        {0, 0},       // Mode 8 (invalid)
        {0, 0}        // Mode 9 (invalid)
    }}}
};

const uint16_t g_maxMode = 9;

std::string g_module = "crosby";
uint16_t g_mode = 1;
uint16_t g_num_frames = 1;
uint16_t g_fps = 10;

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

TEST_F(CameraTestFixture, CameraGetFrames) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    auto it = g_modeDetailsMap.find(g_module);
    ASSERT_TRUE(it != g_modeDetailsMap.end());
    std::array<ModeDetails_struct, 10> crosbyModeDetails = it->second;
    
    // Initialize camera first (might fail in test environment)
    Status init_status = camera->initialize();
    
    if (init_status == Status::OK) {
        Status status = camera->setMode(g_mode);
        ASSERT_TRUE(status == Status::OK);

        status = camera->adsd3500SetFrameRate(g_fps);
        ASSERT_TRUE(status == Status::OK);

        status = camera->start();
        ASSERT_TRUE(status == Status::OK);

        auto startTime = std::chrono::steady_clock::now();
        uint16_t framesReceived = 0;

        for (uint16_t cnt = 0; cnt < g_num_frames; cnt++) {
            aditof::Frame frame;
            status = camera->requestFrame(&frame);
            if (status != Status::OK) {
                status = camera->stop();
                ASSERT_TRUE(status == Status::OK);
            }
            if (status == Status::OK) {
                framesReceived++;
            }
            
            FrameDataDetails fDetails;
            uint16_t *data1;
            status = frame.getData("ab", &data1);
            ASSERT_TRUE(status == Status::OK);
            frame.getDataDetails("ab", fDetails);
            ASSERT_TRUE(fDetails.width == crosbyModeDetails[g_mode].width);
            ASSERT_TRUE(fDetails.height == crosbyModeDetails[g_mode].height);
        }

        auto endTime = std::chrono::steady_clock::now();
        double seconds = std::chrono::duration_cast<std::chrono::duration<double>>(endTime - startTime).count();
        double fps = seconds > 0.0 ? static_cast<double>(framesReceived) / seconds : 0.0;

        status = camera->stop();
        ASSERT_TRUE(status == Status::OK);

        ::testing::Test::RecordProperty("frames_received", framesReceived);
        ::testing::Test::RecordProperty("elapsed_seconds", seconds);
        ::testing::Test::RecordProperty("fps", fps);
        fps = std::round(fps);
        if (g_num_frames > 10) {
            EXPECT_FLOAT_EQ(fps, static_cast<double>(g_fps));
        }
        EXPECT_EQ(framesReceived, g_num_frames);

    } else {
        GTEST_SKIP() << "Camera initialization failed, skipping test";
    }
}

int main(int argc, char** argv) {

    bool bHelp = false;
    // Parse custom command-line argument for expected version
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg.find("--module=") == 0) {
            g_module = arg.substr(9);  // Extract module after "--module="
        } else if (arg.find("--mode=") == 0) {
            g_mode = static_cast<uint16_t>(std::stoi(arg.substr(7)));  // Extract mode after "--mode="
        } else if (arg.find("--num_frames=") == 0) {
            g_num_frames = static_cast<uint16_t>(std::stoi(arg.substr(13)));  // Extract num_frames after "--num_frames="
        } else if (arg.find("--fps=") == 0) {
            g_fps = static_cast<uint16_t>(std::stoi(arg.substr(6)));  // Extract fps after "--fps="
        } else if (arg.find("--ip=") == 0) {
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
        std::cout << "Usage: " << argv[0] << " [--module=<module_name>] [--mode=<mode_number>] [--num_frames=<number_of_frames>] [--fps=<frames_per_second>] [--help|-h]" << std::endl;
        std::cout << "  --module: Specify the camera module (default: crosby)" << std::endl;
        std::cout << "  --mode: Specify the camera mode (default: 1)" << std::endl;
        std::cout << "  --num_frames: Specify the number of frames to capture (default: 1)" << std::endl;
        std::cout << "  --fps: Specify the frames per second (default: 10)" << std::endl;
        std::cout << "  --ip: Specify the camera IP address" << std::endl;
        std::cout << std::endl;
    }
    ::testing::InitGoogleTest(&newArgc, newArgv.data());

    ::testing::Test::RecordProperty("Parameter module", g_module);
    ::testing::Test::RecordProperty("Parameter mode", g_mode);
    ::testing::Test::RecordProperty("Parameter num_frames", g_num_frames);
    ::testing::Test::RecordProperty("Parameter fps", g_fps);

    return RUN_ALL_TESTS();
}