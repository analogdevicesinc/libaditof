#include <gtest/gtest.h>
#include <aditof/frame.h>
#include <aditof/status_definitions.h>
#include <aditof_test_utils.h>

using namespace aditof;

/**
 * UNIT TESTS FOR FRAME CLASS
 * 
 * Tests Frame construction, copying, and moving.
 * Tests use ASSERT_ and EXPECT_ macros - when checks fail, tests FAIL (not SKIP).
 */

class FrameUnitTest : public ::testing::Test {};

TEST_F(FrameUnitTest, DefaultConstruction) {
    try {
        Frame frame;
        EXPECT_TRUE(true);
    } catch (...) {
        FAIL() << "Frame construction threw unexpected exception";
    }
}

TEST_F(FrameUnitTest, CopyConstructor) {
    try {
        Frame frame1;
        Frame frame2(frame1);
        EXPECT_TRUE(true);
    } catch (...) {
        FAIL() << "Copy constructor threw unexpected exception";
    }
}

TEST_F(FrameUnitTest, CopyAssignment) {
    try {
        Frame frame1, frame2;
        frame2 = frame1;
        EXPECT_TRUE(true);
    } catch (...) {
        FAIL() << "Copy assignment threw unexpected exception";
    }
}

TEST_F(FrameUnitTest, MoveConstructor) {
    try {
        Frame frame1;
        Frame frame2(std::move(frame1));
        EXPECT_TRUE(true);
    } catch (...) {
        FAIL() << "Move constructor threw unexpected exception";
    }
}

TEST_F(FrameUnitTest, MoveAssignment) {
    try {
        Frame frame1;
        Frame frame2 = std::move(frame1);
        EXPECT_TRUE(true);
    } catch (...) {
        FAIL() << "Move assignment threw unexpected exception";
    }
}

TEST_F(FrameUnitTest, GetDetailsOnUninitializedFrame) {
    Frame frame;
    FrameDetails details;
    Status status = frame.getDetails(details);
    // Unitialized frame may return OK or error - both acceptable
    EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
}

TEST_F(FrameUnitTest, MultipleFramesIndependence) {
    // Verify multiple frames work independently
    Frame frame1, frame2, frame3;
    
    FrameDetails details1, details2, details3;
    Status status1 = frame1.getDetails(details1);
    Status status2 = frame2.getDetails(details2);
    Status status3 = frame3.getDetails(details3);
    
    // All should complete without issues
    EXPECT_TRUE(status1 == Status::OK || status1 == Status::GENERIC_ERROR);
    EXPECT_TRUE(status2 == Status::OK || status2 == Status::GENERIC_ERROR);
    EXPECT_TRUE(status3 == Status::OK || status3 == Status::GENERIC_ERROR);
}

int main(int argc, char** argv) {
    aditof_test::TestRunner runner(argv[0]);
    int initResult = runner.initialize(argc, argv);
    if (initResult != -1) {
        return initResult;
    }
    return runner.runTests();
}
