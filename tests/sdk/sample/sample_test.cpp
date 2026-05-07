#include <aditof/camera.h>
#include <aditof/camera_definitions.h>
#include <aditof/frame.h>
#include <aditof/status_definitions.h>
#include <aditof/system.h>
#include <aditof/version.h>
#include <gtest/gtest.h>

using namespace aditof;

// Test Frame class
TEST(FrameTest, FrameInstantiation) {
    EXPECT_NO_THROW({ Frame frame; });
}

TEST(FrameTest, FrameDetailsAccess) {
    Frame frame;
    FrameDetails details;

    Status status = frame.getDetails(details);
    // Frame might not be initialized, but the call should not crash
    EXPECT_TRUE(status == Status::OK || status == Status::GENERIC_ERROR);
}

// Test Status enum
TEST(StatusTest, StatusEnumValues) {
    EXPECT_EQ(static_cast<int>(Status::OK), 0);
    EXPECT_NE(static_cast<int>(Status::GENERIC_ERROR), 0);
    EXPECT_NE(static_cast<int>(Status::INVALID_ARGUMENT), 0);
}

// Parameterized test for frame types
class FrameTypeTest : public ::testing::TestWithParam<std::string> {};

TEST_P(FrameTypeTest, FrameTypeStringValid) {
    std::string frameType = GetParam();
    EXPECT_FALSE(frameType.empty());
    // Common frame types should have reasonable length
    EXPECT_LT(frameType.length(), 20);
}

INSTANTIATE_TEST_SUITE_P(CommonFrameTypes, FrameTypeTest,
                         ::testing::Values("depth", "ir", "ab", "xyz", "conf"));
