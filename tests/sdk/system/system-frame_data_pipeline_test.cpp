/**
 * SYSTEM TEST: Frame Data Pipeline
 * 
 * Tests the frame data pipeline:
 * - Frame data types (depth, ab, conf, xyz)
 * - Frame metadata parsing
 * - Frame data integrity
 * - Frame buffer access
 */

#include <gtest/gtest.h>
#include <aditof/system.h>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/frame_definitions.h>
#include <aditof/camera_definitions.h>
#include <aditof/status_definitions.h>
#include <aditof_test_utils.h>
#include <vector>
#include <memory>
#include <cstring>

using namespace aditof;

std::string& g_cameraipaddress = aditof_test::g_cameraipaddress;

/**
 * FrameDataPipelineTest - Tests frame data capture and access
 */
class FrameDataPipelineTest : public ::testing::Test {
protected:
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    std::shared_ptr<Camera> camera;
    bool streamingStarted = false;
    
    void SetUp() override {
        Status status;
        if (g_cameraipaddress.empty()) {
            status = system.getCameraList(cameras);
        } else {
            status = system.getCameraList(cameras, "ip:" + g_cameraipaddress);
        }
        
        if (status != Status::OK || cameras.empty()) {
            GTEST_SKIP_("No camera available for frame data test");
        }
        
        camera = cameras[0];
        
        // Initialize and start camera for frame capture tests
        status = camera->initialize();
        if (status != Status::OK) {
            GTEST_SKIP_("Camera initialization failed");
        }
        
        std::vector<uint8_t> modes;
        status = camera->getAvailableModes(modes);
        if (status != Status::OK || modes.empty()) {
            GTEST_SKIP_("No modes available");
        }
        
        status = camera->setMode(modes[0]);
        if (status != Status::OK) {
            GTEST_SKIP_("setMode failed");
        }
        
        status = camera->start();
        if (status != Status::OK) {
            GTEST_SKIP_("Camera start failed");
        }
        streamingStarted = true;
    }
    
    void TearDown() override {
        if (camera && streamingStarted) {
            camera->stop();
        }
    }
};

// Test: Frame can be captured and has details
TEST_F(FrameDataPipelineTest, FrameCaptureHasDetails) {
    Frame frame;
    Status status = camera->requestFrame(&frame);
    ASSERT_EQ(status, Status::OK);
    
    FrameDetails details;
    status = frame.getDetails(details);
    EXPECT_EQ(status, Status::OK);
}

// Test: Frame has depth data
TEST_F(FrameDataPipelineTest, FrameHasDepthData) {
    Frame frame;
    Status status = camera->requestFrame(&frame);
    ASSERT_EQ(status, Status::OK);
    
    uint16_t* depthData = nullptr;
    status = frame.getData("depth", &depthData);
    
    if (status == Status::OK) {
        EXPECT_NE(depthData, nullptr);
    }
    // Note: depth may not be available depending on mode
}

// Test: Frame has AB (active brightness) data
TEST_F(FrameDataPipelineTest, FrameHasABData) {
    Frame frame;
    Status status = camera->requestFrame(&frame);
    ASSERT_EQ(status, Status::OK);
    
    uint16_t* abData = nullptr;
    status = frame.getData("ab", &abData);
    
    if (status == Status::OK) {
        EXPECT_NE(abData, nullptr);
        
        // Verify data details
        FrameDataDetails abDetails;
        status = frame.getDataDetails("ab", abDetails);
        if (status == Status::OK) {
            EXPECT_GT(abDetails.width, 0u);
            EXPECT_GT(abDetails.height, 0u);
        }
    }
}

// Test: Frame has confidence data
TEST_F(FrameDataPipelineTest, FrameHasConfidenceData) {
    Frame frame;
    Status status = camera->requestFrame(&frame);
    ASSERT_EQ(status, Status::OK);
    
    uint16_t* confData = nullptr;
    status = frame.getData("conf", &confData);
    
    // Confidence data may or may not be available
    if (status == Status::OK) {
        EXPECT_NE(confData, nullptr);
    }
}

// Test: Frame data details match expected dimensions
TEST_F(FrameDataPipelineTest, FrameDataDimensionsConsistent) {
    Frame frame;
    Status status = camera->requestFrame(&frame);
    ASSERT_EQ(status, Status::OK);
    
    FrameDetails frameDetails;
    status = frame.getDetails(frameDetails);
    ASSERT_EQ(status, Status::OK);
    
    // Check if data details are consistent
    for (const auto& dataDetail : frameDetails.dataDetails) {
        EXPECT_GT(dataDetail.width, 0u) << "Invalid width for " << dataDetail.type;
        EXPECT_GT(dataDetail.height, 0u) << "Invalid height for " << dataDetail.type;
    }
}

// Test: Frame metadata is accessible
TEST_F(FrameDataPipelineTest, FrameMetadataAccessible) {
    Frame frame;
    Status status = camera->requestFrame(&frame);
    ASSERT_EQ(status, Status::OK);
    
    Metadata metadata;
    status = frame.getMetadataStruct(metadata);
    
    if (status == Status::OK) {
        // Basic metadata validation
        EXPECT_GT(metadata.width, 0u);
        EXPECT_GT(metadata.height, 0u);
    }
}

// Test: Frame metadata frame number increments
TEST_F(FrameDataPipelineTest, FrameMetadataFrameNumberIncrements) {
    Frame frame1, frame2;
    
    Status status = camera->requestFrame(&frame1);
    ASSERT_EQ(status, Status::OK);
    
    status = camera->requestFrame(&frame2);
    ASSERT_EQ(status, Status::OK);
    
    Metadata meta1, meta2;
    Status s1 = frame1.getMetadataStruct(meta1);
    Status s2 = frame2.getMetadataStruct(meta2);
    
    if (s1 == Status::OK && s2 == Status::OK) {
        // Frame numbers should be sequential or at least increasing
        EXPECT_GE(meta2.frameNumber, meta1.frameNumber);
    }
}

// Test: Multiple consecutive frame captures
TEST_F(FrameDataPipelineTest, ConsecutiveFrameCaptures) {
    const int NUM_FRAMES = 10;
    int successCount = 0;
    
    for (int i = 0; i < NUM_FRAMES; i++) {
        Frame frame;
        Status status = camera->requestFrame(&frame);
        if (status == Status::OK) {
            successCount++;
            
            // Verify each frame has valid data
            FrameDetails details;
            frame.getDetails(details);
        }
    }
    
    EXPECT_EQ(successCount, NUM_FRAMES) << "Not all frames captured successfully";
}

// Test: Frame data type checking
TEST_F(FrameDataPipelineTest, FrameHaveDataTypeCheck) {
    Frame frame;
    Status status = camera->requestFrame(&frame);
    ASSERT_EQ(status, Status::OK);
    
    // Check common data types
    std::vector<std::string> dataTypes = {"depth", "ab", "conf", "xyz"};
    
    for (const auto& type : dataTypes) {
        bool hasType = frame.haveDataType(type);
        // Record which types are available
        ::testing::Test::RecordProperty(("Has_" + type).c_str(), hasType ? "yes" : "no");
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
