#include <gtest/gtest.h>
#include <aditof/version.h>
#include <aditof/system.h>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/camera_definitions.h>
#include <aditof/status_definitions.h>
#include <string>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <vector>

// Use: ./sdk_version_test --version=6.2.0

using namespace aditof;

// Global variable to store the expected version from command line
std::string g_expectedVersion = "6.2.0";

// Generate UTC timestamp in format: YYYYMMDD_HHMMSS
std::string getUTCTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    
    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&time_t_now), "%Y%m%d_%H%M%S");
    return oss.str();
}

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
    bool bHelp = false;
    // Parse custom command-line argument for expected version
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg.find("--version=") == 0) {
            g_expectedVersion = arg.substr(10);  // Extract version after "--version="
        }  else if (arg == "--help" || arg == "-h") {
            bHelp = true;
        }
    }

    // Automatically add --gtest_output with timestamped filename
    std::string timestamp = getUTCTimestamp();
    std::string gtestOutput = "--gtest_output=json:report_" + timestamp + ".json";
    
    // Create new argv with the additional argument
    std::vector<char*> newArgv;
    for (int i = 0; i < argc; ++i) {
        newArgv.push_back(argv[i]);
    }
    newArgv.push_back(const_cast<char*>(gtestOutput.c_str()));
    newArgv.push_back(nullptr);
    
    int newArgc = argc + 1;

    if (bHelp) {
        std::cout << "Usage: " << argv[0] << " [--version=<expected_version>] [--help|-h]" << std::endl;
        std::cout << "  --version: Specify the expected API version (default: 6.2.0)" << std::endl;
        std::cout << std::endl;
    }
    ::testing::InitGoogleTest(&newArgc, newArgv.data());

    ::testing::Test::RecordProperty("Parameter expected_version", g_expectedVersion);

    return RUN_ALL_TESTS();
}