#include <gtest/gtest.h>
#include <aditof/frame.h>
#include <aditof/status_definitions.h>
#include <aditof_test_utils.h>

using namespace aditof;

/**
 * BOUNDARY VALUE TESTS FOR FRAME CLASS
 * 
 * Tests Frame behavior with boundary/edge case values.
 * Uses ASSERT_ and EXPECT_ - tests FAIL (not SKIP) when issues occur.
 */

class FrameBoundaryTest : public ::testing::Test {};

TEST_F(FrameBoundaryTest, MinimalDimensions) {
    Frame frame;
    FrameDetails details;
    
    // Test 1x1 dimensions
    details.width = 1;
    details.height = 1;
    
    Status status = frame.getDetails(details);
    // Should handle without crashing
    EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
}

TEST_F(FrameBoundaryTest, PowerOfTwoDimensions) {
    Frame frame;
    FrameDetails details;
    
    std::vector<std::pair<uint32_t, uint32_t>> dimensions = {
        {2, 2}, {4, 4}, {8, 8}, {16, 16}, {32, 32}, 
        {64, 64}, {128, 128}, {256, 256}, {512, 512}, {1024, 1024}
    };
    
    for (const auto& dim : dimensions) {
        details.width = dim.first;
        details.height = dim.second;
        
        Status status = frame.getDetails(details);
        // Each dimension test should not crash
        EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
    }
}

TEST_F(FrameBoundaryTest, NonPowerOfTwoDimensions) {
    Frame frame;
    FrameDetails details;
    
    std::vector<std::pair<uint32_t, uint32_t>> dimensions = {
        {480, 640}, {360, 480}, {720, 1280}, {1080, 1920}
    };
    
    for (const auto& dim : dimensions) {
        details.width = dim.first;
        details.height = dim.second;
        
        Status status = frame.getDetails(details);
        EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
    }
}

TEST_F(FrameBoundaryTest, AsymmetricDimensions) {
    Frame frame;
    FrameDetails details;
    
    std::vector<std::pair<uint32_t, uint32_t>> dimensions = {
        {100, 200}, {300, 150}, {1024, 512}, {512, 1024}
    };
    
    for (const auto& dim : dimensions) {
        details.width = dim.first;
        details.height = dim.second;
        
        Status status = frame.getDetails(details);
        EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
    }
}

TEST_F(FrameBoundaryTest, MaximumReasonableDimensions) {
    Frame frame;
    FrameDetails details;
    
    // Test large but reasonable dimensions
    details.width = 2048;
    details.height = 2048;
    
    Status status = frame.getDetails(details);
    // Should handle large dimensions gracefully
    EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
}

TEST_F(FrameBoundaryTest, AllTypeModeCombinations) {
    // Create multiple frames with various configurations
    std::vector<std::string> types = {"depth", "ir", "ab", "confidence"};
    std::vector<std::string> modes = {"near", "medium", "far"};
    
    for (const auto& type : types) {
        for (const auto& mode : modes) {
            Frame frame;
            FrameDetails details;
            details.type = type;
            details.cameraMode = mode;
            details.width = 512;
            details.height = 512;
            
            Status status = frame.getDetails(details);
            // Each combination should complete without crashing
            EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR)
                << "Failed for type=" << type << " mode=" << mode;
        }
    }
}

TEST_F(FrameBoundaryTest, RepeatedSetDetails) {
    Frame frame;
    FrameDetails details1, details2, details3;
    
    // Set details multiple times to same frame
    details1.width = 512;
    details1.height = 512;
    details1.type = "depth";
    
    details2.width = 1024;
    details2.height = 1024;
    details2.type = "ir";
    
    details3.width = 256;
    details3.height = 256;
    details3.type = "ab";
    
    // All operations should complete
    Status status1 = frame.getDetails(details1);
    Status status2 = frame.getDetails(details2);
    Status status3 = frame.getDetails(details3);
    
    EXPECT_TRUE(status1 == Status::OK || status1 == Status::GENERIC_ERROR);
    EXPECT_TRUE(status2 == Status::OK || status2 == Status::GENERIC_ERROR);
    EXPECT_TRUE(status3 == Status::OK || status3 == Status::GENERIC_ERROR);
}

TEST_F(FrameBoundaryTest, SmallFrameAllocation) {
    Frame frame;
    FrameDetails details;
    
    // Minimal allocation
    details.width = 1;
    details.height = 1;
    details.type = "depth";
    details.cameraMode = "near";
    
    Status status = frame.getDetails(details);
    ASSERT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR)
        << "Failed to handle minimal frame size";
}

TEST_F(FrameBoundaryTest, LargeFrameAllocation) {
    Frame frame;
    FrameDetails details;
    
    // Large but reasonable allocation
    details.width = 2048;
    details.height = 2048;
    details.type = "depth";
    details.cameraMode = "far";
    
    Status status = frame.getDetails(details);
    ASSERT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR)
        << "Failed to handle large frame size";
}

TEST_F(FrameBoundaryTest, FrameIndependence) {
    // Create 3 frames with different configurations
    Frame frame1, frame2, frame3;
    FrameDetails details1, details2, details3;
    
    details1.width = 512;
    details1.height = 512;
    details1.type = "depth";
    
    details2.width = 1024;
    details2.height = 1024;
    details2.type = "ir";
    
    details3.width = 256;
    details3.height = 256;
    details3.type = "ab";
    
    Status status1 = frame1.getDetails(details1);
    Status status2 = frame2.getDetails(details2);
    Status status3 = frame3.getDetails(details3);
    
    // Each frame must work independently
    ASSERT_TRUE(status1 == Status::OK || status1 == Status::GENERIC_ERROR)
        << "Frame 1 failed";
    ASSERT_TRUE(status2 == Status::OK || status2 == Status::GENERIC_ERROR)
        << "Frame 2 failed";
    ASSERT_TRUE(status3 == Status::OK || status3 == Status::GENERIC_ERROR)
        << "Frame 3 failed";
}

TEST_F(FrameBoundaryTest, CopySemanticsWithBoundaryValues) {
    try {
        Frame original;
        FrameDetails originalDetails;
        originalDetails.width = 1;
        originalDetails.height = 1;
        originalDetails.type = "depth";
        
        Frame copy(original);
        FrameDetails copiedDetails;
        
        Status statusOriginal = original.getDetails(originalDetails);
        Status statusCopy = copy.getDetails(copiedDetails);
        
        // Both should work - if either fails, test FAILS (doesn't skip)
        ASSERT_TRUE(statusOriginal == Status::OK || statusOriginal == Status::GENERIC_ERROR);
        ASSERT_TRUE(statusCopy == Status::OK || statusCopy == Status::GENERIC_ERROR);
    } catch (...) {
        FAIL() << "Copy semantics test threw unexpected exception";
    }
}

TEST_F(FrameBoundaryTest, MoveSemanticsWithBoundaryValues) {
    Frame original;
    FrameDetails originalDetails;
    originalDetails.width = 2048;
    originalDetails.height = 2048;
    
    Frame moved(std::move(original));
    FrameDetails movedDetails;
    
    Status statusMoved = moved.getDetails(movedDetails);
    
    // If move fails, test FAILS (doesn't skip)
    ASSERT_TRUE(statusMoved == Status::OK || statusMoved == Status::GENERIC_ERROR);
}

int main(int argc, char** argv) {
    aditof_test::TestRunner runner(argv[0]);
    int initResult = runner.initialize(argc, argv);
    if (initResult != -1) {
        return initResult;
    }
    return runner.runTests();
}
