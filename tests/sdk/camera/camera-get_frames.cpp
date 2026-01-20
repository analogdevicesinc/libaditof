#include <gtest/gtest.h>
#include <aditof/version.h>
#include <aditof/system.h>
#include <aditof_test_utils.h>
#include <aditof/frame_handler.h>
#include <iostream>
#include <vector>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/camera_definitions.h>
#include <aditof/status_definitions.h>
#include <array>
#include <map>
#include <cmath>
#include <fstream>
#include <random>

using namespace aditof;

// Use the camera IP address from aditof_test library
std::string& g_cameraipaddress = aditof_test::g_cameraipaddress;
bool g_savelastframe = false;
std::string g_timestamp;

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
uint16_t g_mode;
uint16_t g_num_frames = 1;
uint16_t g_fps = 10;
uint16_t g_modeorder = 0;

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

#include <fstream>
Status save_frames(const std::string testname,
                  aditof::Frame &frame,
                  const int &mode_num) {

    const std::string filename = testname + "_mode_" + std::to_string(mode_num);
    FrameHandler fh;
    return fh.SnapShotFrames(filename.c_str(), &frame, nullptr, nullptr);
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

        std::vector<uint8_t> available_modes;
        Status status = camera->getAvailableModes(available_modes);
        ASSERT_TRUE(status == Status::OK);
        ASSERT_FALSE(available_modes.empty());

        // If g_mode is empty, randomly choose from available modes
        if (g_mode == 0) {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> dis(0, available_modes.size() - 1);
            g_mode = available_modes[dis(gen)];
        }

        ::testing::Test::RecordProperty("mode", g_mode);

        status = camera->setMode(g_mode);
        ASSERT_TRUE(status == Status::OK);

        status = camera->adsd3500SetFrameRate(g_fps);
        ASSERT_TRUE(status == Status::OK);

        status = camera->start();
        ASSERT_TRUE(status == Status::OK);

        auto startTime = std::chrono::steady_clock::now();
        uint16_t framesReceived = 0;
        uint32_t frameNumber = 0;

        aditof::Frame frame;
        for (uint16_t cnt = 0; cnt < g_num_frames; cnt++) {
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

            aditof::Metadata metadata;
            status = frame.getMetadataStruct(metadata);
            ASSERT_TRUE(status == Status::OK);

            if (frameNumber == 0) {
                frameNumber = metadata.frameNumber;
            } else {
                if (metadata.frameNumber != frameNumber + 1) {
                    ADD_FAILURE() << "Expected Frame Number: " << frameNumber + 1 << " Got: " << metadata.frameNumber;
                }
                frameNumber = metadata.frameNumber;
            }

            ASSERT_TRUE(metadata.imagerMode == g_mode);
            ASSERT_TRUE(metadata.width == crosbyModeDetails[g_mode].width);
            ASSERT_TRUE(metadata.height == crosbyModeDetails[g_mode].height);
        }

        auto endTime = std::chrono::steady_clock::now();
        double seconds = std::chrono::duration_cast<std::chrono::duration<double>>(endTime - startTime).count();
        double fps = seconds > 0.0 ? static_cast<double>(framesReceived) / seconds : 0.0;

        status = camera->stop();
        ASSERT_TRUE(status == Status::OK);

        if (g_savelastframe && framesReceived > 0) {
            status = save_frames("CameraGetFrames", frame, g_mode);
            ASSERT_TRUE(status == Status::OK);
        }

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

TEST_F(CameraTestFixture, CameraGetFramesAllModes) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    auto it = g_modeDetailsMap.find(g_module);
    ASSERT_TRUE(it != g_modeDetailsMap.end());
    std::array<ModeDetails_struct, 10> crosbyModeDetails = it->second;
    
    // Initialize camera first (might fail in test environment)
    Status init_status = camera->initialize();
    
    if (init_status == Status::OK) {

        std::vector<uint8_t> available_modes;
        Status status = camera->getAvailableModes(available_modes);
        ASSERT_TRUE(status == Status::OK);
        ASSERT_FALSE(available_modes.empty());
        
        if (g_modeorder == 0) {
            // Keep modes in ascending order
            std::sort(available_modes.begin(), available_modes.end());
        } else if (g_modeorder == 1) {
            // Keep modes in descending order
            std::sort(available_modes.begin(), available_modes.end(), std::greater<uint8_t>());
        } else {
            // Shuffle modes to randomly test them without repetition
            std::random_device rd;
            std::mt19937 gen(rd());
            std::shuffle(available_modes.begin(), available_modes.end(), gen);
        }

        for (const auto& mode : available_modes) {
            g_mode = mode;

            ::testing::Test::RecordProperty("mode_" + std::to_string(g_mode), g_mode);

            status = camera->setMode(g_mode);
            ASSERT_TRUE(status == Status::OK);

            status = camera->adsd3500SetFrameRate(g_fps);
            ASSERT_TRUE(status == Status::OK);

            status = camera->start();
            ASSERT_TRUE(status == Status::OK);

            auto startTime = std::chrono::steady_clock::now();
            uint16_t framesReceived = 0;
            uint32_t frameNumber = 0;

            aditof::Frame frame;
            for (uint16_t cnt = 0; cnt < g_num_frames; cnt++) {
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

                aditof::Metadata metadata;
                status = frame.getMetadataStruct(metadata);
                ASSERT_TRUE(status == Status::OK);

                if (frameNumber == 0) {
                    frameNumber = metadata.frameNumber;
                } else {
                    if (metadata.frameNumber != frameNumber + 1) {
                        ADD_FAILURE() << "Expected Frame Number: " << frameNumber + 1 << " Got: " << metadata.frameNumber;
                    }
                    frameNumber = metadata.frameNumber;
                }

                ASSERT_TRUE(metadata.imagerMode == g_mode);
                ASSERT_TRUE(metadata.width == crosbyModeDetails[g_mode].width);
                ASSERT_TRUE(metadata.height == crosbyModeDetails[g_mode].height);
            }

            auto endTime = std::chrono::steady_clock::now();
            double seconds = std::chrono::duration_cast<std::chrono::duration<double>>(endTime - startTime).count();
            double fps = seconds > 0.0 ? static_cast<double>(framesReceived) / seconds : 0.0;

            status = camera->stop();
            ASSERT_TRUE(status == Status::OK);

            if (g_savelastframe && framesReceived > 0) {
                status = save_frames("CameraGetFramesAllModes", frame, g_mode);
                ASSERT_TRUE(status == Status::OK);
            }

            ::testing::Test::RecordProperty("frames_received_" + std::to_string(g_mode), framesReceived);
            ::testing::Test::RecordProperty("elapsed_seconds_" + std::to_string(g_mode), seconds);
            ::testing::Test::RecordProperty("fps_" + std::to_string(g_mode), fps);
            fps = std::round(fps);
            if (g_num_frames > 10) {
                EXPECT_FLOAT_EQ(fps, static_cast<double>(g_fps));
            }
            EXPECT_EQ(framesReceived, g_num_frames);
        }
    } else {
        GTEST_SKIP() << "Camera initialization failed, skipping test";
    }
}

int main(int argc, char** argv) {
    // Create test runner
    aditof_test::TestRunner runner(argv[0]);
    
    // Add custom arguments
    runner.addArgument({"--module=", &g_module, "Specify the camera module (default: crosby)"});
    runner.addArgument({"--mode=", &g_mode, "Specify the camera mode (default: 1)"});
    runner.addArgument({"--frames=", &g_num_frames, "Specify the number of frames to capture (default: 1)"});
    runner.addArgument({"--fps=", &g_fps, "Specify the frames per second (default: 10)"});
    runner.addArgument({"--save", &g_savelastframe, "Save the last captured frame to a binary file"});
    runner.addArgument({"--order=", &g_modeorder, "Order for modes (deftault: 0 - ascending, 1 - descending, 2 - random)"});
    // Note: --ip argument is automatically added by TestRunner
    
    // Initialize (parses args, sets up GTest output)
    int initResult = runner.initialize(argc, argv);
    if (initResult != -1) {
        return initResult;  // Help was shown or error occurred
    }
    
    // Set timestamp for save_frames function
    g_timestamp = runner.getTimestamp();
    
    // Run tests
    return runner.runTests();
}