#include <gtest/gtest.h>
#include <aditof/version.h>
#include <aditof/system.h>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/camera_definitions.h>
#include <aditof/status_definitions.h>
#include <aditof_test_utils.h>
#include <string>
#include <vector>

// Use: ./sdk_version_test --version=6.2.0

using namespace aditof;

// Global variable to store the expected version from command line
std::string g_expectedVersion = "7.0.0 a-1";

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

    ::testing::Test::RecordProperty("Expected Version: ", g_expectedVersion);
    ::testing::Test::RecordProperty("Read Version: ", version);
    EXPECT_TRUE(version == g_expectedVersion);
}

int main(int argc, char** argv) {
    // Create test runner
    aditof_test::TestRunner runner(argv[0]);
    
    // Add custom arguments
    runner.addArgument({"--version=", &g_expectedVersion, "Specify the expected API version (default: 6.2.0)"});
    
    // Initialize (parses args, sets up GTest output)
    int initResult = runner.initialize(argc, argv);
    if (initResult != -1) {
        return initResult;  // Help was shown or error occurred
    }
    
    // Run tests
    return runner.runTests();
}