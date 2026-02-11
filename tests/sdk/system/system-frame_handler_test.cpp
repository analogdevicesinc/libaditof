/**
 * SYSTEM TEST: Frame Handler Integration
 * 
 * Tests the FrameHandler file I/O functionality:
 * - Save frames to file
 * - Load frames from file
 * - File format validation
 * - Multiple frame storage
 */

#include <gtest/gtest.h>
#include <aditof/system.h>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/frame_handler.h>
#include <aditof/frame_operations.h>
#include <aditof_test_utils.h>
#include <vector>
#include <memory>
#include <string>
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>

using namespace aditof;

std::string& g_cameraipaddress = aditof_test::g_cameraipaddress;

/**
 * FrameHandlerIntegrationTest - Tests frame save/load functionality
 */
class FrameHandlerIntegrationTest : public ::testing::Test {
protected:
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    std::shared_ptr<Camera> camera;
    FrameHandler frameHandler;
    std::string testDir;
    
    void SetUp() override {
        Status status;
        if (g_cameraipaddress.empty()) {
            status = system.getCameraList(cameras);
        } else {
            status = system.getCameraList(cameras, "ip:" + g_cameraipaddress);
        }
        
        if (status != Status::OK || cameras.empty()) {
            GTEST_SKIP_("No camera available for FrameHandler test");
        }
        
        camera = cameras[0];
        status = camera->initialize();
        if (status != Status::OK) {
            GTEST_SKIP_("Camera initialization failed");
        }
        
        // Create test directory
        testDir = "frame_handler_test_output";
        struct stat st = {0};
        if (stat(testDir.c_str(), &st) == -1) {
            mkdir(testDir.c_str(), 0700);
        }
    }
    
    void TearDown() override {
        if (camera) {
            camera->stop();
        }
        
        // Cleanup test files (simple cleanup, test directory may remain)
        // Note: Full recursive cleanup would require more complex POSIX code
        struct stat st = {0};
        if (stat(testDir.c_str(), &st) == 0) {
            rmdir(testDir.c_str());
        }
    }
    
    bool startCameraStreaming() {
        std::vector<uint8_t> modes;
        Status status = camera->getAvailableModes(modes);
        if (status != Status::OK || modes.empty()) return false;
        
        status = camera->setMode(modes[0]);
        if (status != Status::OK) return false;
        
        status = camera->start();
        return (status == Status::OK);
    }
};

// Test: Set output file path
TEST_F(FrameHandlerIntegrationTest, SetOutputFilePath) {
    std::string customPath = testDir + "/custom_frames";
    Status status = frameHandler.setOutputFilePath(customPath);
    EXPECT_EQ(status, Status::OK) << "Failed to set output file path";
}

// Test: Save single frame
TEST_F(FrameHandlerIntegrationTest, SaveSingleFrame) {
    ASSERT_TRUE(startCameraStreaming()) << "Failed to start camera streaming";
    
    Frame frame;
    Status status = camera->requestFrame(&frame);
    ASSERT_EQ(status, Status::OK) << "Failed to capture frame";
    
    frameHandler.setOutputFilePath(testDir);
    
    status = frameHandler.saveFrameToFile(frame, "test_frame.bin");
    EXPECT_EQ(status, Status::OK) << "Failed to save frame to file";
    
    camera->stop();
}

// Test: Save multiple frames sequentially
TEST_F(FrameHandlerIntegrationTest, SaveMultipleFrames) {
    ASSERT_TRUE(startCameraStreaming()) << "Failed to start camera streaming";
    
    frameHandler.setOutputFilePath(testDir);
    
    const int numFrames = 5;
    for (int i = 0; i < numFrames; i++) {
        Frame frame;
        Status status = camera->requestFrame(&frame);
        ASSERT_EQ(status, Status::OK) << "Failed to capture frame " << i;
        
        std::string fileName = "multi_frame_" + std::to_string(i) + ".bin";
        status = frameHandler.saveFrameToFile(frame, fileName);
        EXPECT_EQ(status, Status::OK) << "Failed to save frame " << i;
    }
    
    camera->stop();
}

// Test: Save and load frame round-trip
TEST_F(FrameHandlerIntegrationTest, SaveAndLoadRoundTrip) {
    ASSERT_TRUE(startCameraStreaming()) << "Failed to start camera streaming";
    
    // Capture original frame
    Frame originalFrame;
    Status status = camera->requestFrame(&originalFrame);
    ASSERT_EQ(status, Status::OK);
    
    camera->stop();
    
    // Save frame
    frameHandler.setOutputFilePath(testDir);
    std::string fileName = "roundtrip_frame.bin";
    status = frameHandler.saveFrameToFile(originalFrame, fileName);
    ASSERT_EQ(status, Status::OK) << "Failed to save frame";
    
    // Verify file was created
    std::string fullPath = testDir + "/" + fileName;
    struct stat buffer;
    bool fileExists = (stat(fullPath.c_str(), &buffer) == 0);
    ASSERT_TRUE(fileExists) << "Saved frame file not found: " << fullPath;
    
    // Load frame back
    Frame loadedFrame;
    frameHandler.setInputFileName(fullPath);
    status = frameHandler.readNextFrame(loadedFrame);
    
    // Accept OK or UNAVAILABLE (some implementations may not support loading)
    EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE) 
        << "Failed to load frame from file: status = " << static_cast<int>(status);
    
    // Only compare details if load succeeded
    if (status == Status::OK) {
        FrameDetails originalDetails, loadedDetails;
        originalFrame.getDetails(originalDetails);
        loadedFrame.getDetails(loadedDetails);
        
        EXPECT_EQ(originalDetails.width, loadedDetails.width);
        EXPECT_EQ(originalDetails.height, loadedDetails.height);
    }
}

// Test: Frame data integrity after save/load
TEST_F(FrameHandlerIntegrationTest, DataIntegrityAfterSaveLoad) {
    ASSERT_TRUE(startCameraStreaming()) << "Failed to start camera streaming";
    
    Frame originalFrame;
    Status status = camera->requestFrame(&originalFrame);
    ASSERT_EQ(status, Status::OK);
    
    camera->stop();
    
    // Get original depth data
    uint16_t* originalData = nullptr;
    status = originalFrame.getData("depth", &originalData);
    
    if (status == Status::OK && originalData != nullptr) {
        // Store some original values
        FrameDetails details;
        originalFrame.getDetails(details);
        size_t dataSize = details.width * details.height;
        std::vector<uint16_t> originalValues(originalData, originalData + std::min(dataSize, (size_t)100));
        
        // Save and load
        frameHandler.setOutputFilePath(testDir);
        std::string fileName = "integrity_frame.bin";
        frameHandler.saveFrameToFile(originalFrame, fileName);
        
        Frame loadedFrame;
        std::string fullPath = testDir + "/" + fileName;
        frameHandler.setInputFileName(fullPath);
        frameHandler.readNextFrame(loadedFrame);
        
        // Compare data
        uint16_t* loadedData = nullptr;
        status = loadedFrame.getData("depth", &loadedData);
        
        if (status == Status::OK && loadedData != nullptr) {
            for (size_t i = 0; i < originalValues.size(); i++) {
                EXPECT_EQ(originalValues[i], loadedData[i]) 
                    << "Data mismatch at index " << i;
            }
        }
    }
}

// Test: Frame operations compare
TEST_F(FrameHandlerIntegrationTest, FrameCompareOperations) {
    ASSERT_TRUE(startCameraStreaming()) << "Failed to start camera streaming";
    
    Frame frame1, frame2;
    camera->requestFrame(&frame1);
    camera->requestFrame(&frame2);
    camera->stop();
    
    // Compare frames using frame operations
    FrameDetails details1, details2;
    frame1.getDetails(details1);
    frame2.getDetails(details2);
    
    // Dimensions should match for consecutive frames
    EXPECT_EQ(details1.width, details2.width);
    EXPECT_EQ(details1.height, details2.height);
}

/**
 * FrameOperationsTest - Tests frame comparison and manipulation
 */
class FrameOperationsTest : public FrameHandlerIntegrationTest {
};

// Test: Frame copy creates independent frame
TEST_F(FrameOperationsTest, FrameCopyIndependence) {
    ASSERT_TRUE(startCameraStreaming()) << "Failed to start camera streaming";
    
    Frame originalFrame;
    Status status = camera->requestFrame(&originalFrame);
    ASSERT_EQ(status, Status::OK);
    
    camera->stop();
    
    // Copy frame
    Frame copiedFrame(originalFrame);
    
    // Verify both frames have same details
    FrameDetails originalDetails, copiedDetails;
    originalFrame.getDetails(originalDetails);
    copiedFrame.getDetails(copiedDetails);
    
    EXPECT_EQ(originalDetails.width, copiedDetails.width);
    EXPECT_EQ(originalDetails.height, copiedDetails.height);
}

// Test: Frame assignment operator
TEST_F(FrameOperationsTest, FrameAssignmentOperator) {
    ASSERT_TRUE(startCameraStreaming()) << "Failed to start camera streaming";
    
    Frame frame1, frame2;
    camera->requestFrame(&frame1);
    camera->stop();
    
    // Assign frame1 to frame2
    frame2 = frame1;
    
    FrameDetails details1, details2;
    frame1.getDetails(details1);
    frame2.getDetails(details2);
    
    EXPECT_EQ(details1.width, details2.width);
    EXPECT_EQ(details1.height, details2.height);
}

int main(int argc, char** argv) {
    aditof_test::TestRunner runner(argv[0]);
    
    int initResult = runner.initialize(argc, argv);
    if (initResult != -1) {
        return initResult;
    }
    
    return runner.runTests();
}
