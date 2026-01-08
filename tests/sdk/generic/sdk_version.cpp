#include <gtest/gtest.h>
#include <aditof/version.h>
#include <aditof/system.h>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/camera_definitions.h>
#include <aditof/status_definitions.h>
#include <string>

// Use: ./sdk_version_test --version=6.2.0

using namespace aditof;

// Global variable to store the expected version from command line
std::string g_expectedVersion = "6.2.0";

// Test version information
TEST(VersionTest, VersionMacrosAreDefined) {
    // Version macros are strings, just verify they're not empty
    std::string major(ADITOF_API_VERSION_MAJOR);
    std::string minor(ADITOF_API_VERSION_MINOR);
    std::string patch(ADITOF_API_VERSION_PATCH);
    
    EXPECT_FALSE(major.empty());
    EXPECT_FALSE(minor.empty());
    EXPECT_FALSE(patch.empty());
}

TEST(VersionTest, ApiVersionReturnsNonEmptyString) {
    std::string version = getApiVersion();
    EXPECT_FALSE(version.empty());
    // Version should contain dots
    EXPECT_NE(version.find('.'), std::string::npos);

    EXPECT_TRUE(version == g_expectedVersion);
}

int main(int argc, char** argv) {
    // Parse custom command-line argument for expected version
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg.find("--version=") == 0) {
            g_expectedVersion = arg.substr(10);  // Extract version after "--version="
        }
    }

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}