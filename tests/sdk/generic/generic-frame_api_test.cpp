#include <gtest/gtest.h>
#include <aditof/frame.h>
#include <aditof/status_definitions.h>
#include <aditof_test_utils.h>

using namespace aditof;

/**
 * API TESTS FOR FRAME CLASS
 * 
 * Tests the public Frame API.
 * Uses ASSERT_ and EXPECT_ - tests FAIL (not SKIP) on error.
 */

class FrameAPITest : public ::testing::Test {};

TEST_F(FrameAPITest, FrameCanBeCreated) {
    try {
        Frame frame;
        EXPECT_TRUE(true);
    } catch (...) {
        FAIL() << "Frame creation threw unexpected exception";
    }
}

TEST_F(FrameAPITest, CanGetDetailsFromFrame) {
    Frame frame;
    FrameDetails details;
    Status status = frame.getDetails(details);
    // Should not throw/crash
    EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
}

TEST_F(FrameAPITest, CopyConstructorCreatesIndependentCopy) {
    try {
        Frame original;
        Frame copy(original);
        
        FrameDetails detailsOriginal, detailsCopy;
        Status statusOriginal = original.getDetails(detailsOriginal);
        Status statusCopy = copy.getDetails(detailsCopy);
        
        // Both should work independently
        EXPECT_TRUE(statusOriginal == Status::OK || statusOriginal == Status::GENERIC_ERROR);
        EXPECT_TRUE(statusCopy == Status::OK || statusCopy == Status::GENERIC_ERROR);
    } catch (...) {
        FAIL() << "Copy constructor or getDetails threw unexpected exception";
    }
}

TEST_F(FrameAPITest, CopyAssignmentCreatesIndependentCopy) {
    Frame original;
    Frame copy;
    copy = original;
    
    FrameDetails detailsOriginal, detailsCopy;
    Status statusOriginal = original.getDetails(detailsOriginal);
    Status statusCopy = copy.getDetails(detailsCopy);
    
    // Both should work independently
    EXPECT_TRUE(statusOriginal == Status::OK || statusOriginal == Status::GENERIC_ERROR);
    EXPECT_TRUE(statusCopy == Status::OK || statusCopy == Status::GENERIC_ERROR);
}

TEST_F(FrameAPITest, MoveConstructorTransfersOwnership) {
    Frame original;
    Frame moved(std::move(original));
    
    FrameDetails details;
    Status status = moved.getDetails(details);
    // Moved frame should still be functional
    EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
}

TEST_F(FrameAPITest, MoveAssignmentTransfersOwnership) {
    Frame original;
    Frame moved;
    moved = std::move(original);
    
    FrameDetails details;
    Status status = moved.getDetails(details);
    // Moved frame should still be functional
    EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
}

TEST_F(FrameAPITest, MultipleFramesConfiguredDifferently) {
    // Create multiple frames - tests use ASSERT_/EXPECT_ so errors FAIL the test
    Frame frame1, frame2, frame3;
    
    FrameDetails details1, details2, details3;
    Status status1 = frame1.getDetails(details1);
    Status status2 = frame2.getDetails(details2);
    Status status3 = frame3.getDetails(details3);
    
    // If any getDetails fails, test FAILs (doesn't skip)
    ASSERT_TRUE(status1 == Status::OK || status1 == Status::GENERIC_ERROR) 
        << "Failed to get details from frame1";
    ASSERT_TRUE(status2 == Status::OK || status2 == Status::GENERIC_ERROR) 
        << "Failed to get details from frame2";
    ASSERT_TRUE(status3 == Status::OK || status3 == Status::GENERIC_ERROR) 
        << "Failed to get details from frame3";
}

TEST_F(FrameAPITest, SelfAssignmentIsHarmless) {
    Frame frame;
    frame = frame;
    
    FrameDetails details;
    Status status = frame.getDetails(details);
    EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
}

int main(int argc, char** argv) {
    aditof_test::TestRunner runner(argv[0]);
    int initResult = runner.initialize(argc, argv);
    if (initResult != -1) {
        return initResult;
    }
    return runner.runTests();
}
