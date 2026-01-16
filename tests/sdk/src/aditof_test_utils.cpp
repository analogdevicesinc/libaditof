#include "aditof_test_utils.h"
#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <cstdlib>
#include <climits>

namespace aditof_test {

// Global camera IP address
std::string g_cameraipaddress = "";

std::string getUTCTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    
    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&time_t_now), "%Y%m%d_%H%M%S");
    return oss.str();
}

TestRunner::TestRunner(const std::string& programName)
    : programName_(programName)
    , execName_(programName)
    , helpRequested_(false)
    , strictArgs_(true)
    , preTestValidator_(nullptr)
    , newArgc_(0)
    , executablePath_(programName) {
    
    // Extract just the executable name without path
    size_t lastSlash = execName_.find_last_of("/\\");
    if (lastSlash != std::string::npos) {
        execName_ = execName_.substr(lastSlash + 1);
    }
    
    // Resolve full executable path and extract directory
    char resolvedPath[PATH_MAX];
    if (realpath(programName.c_str(), resolvedPath) != nullptr) {
        std::string fullPath(resolvedPath);
        // Remove executable name, keep only directory
        size_t dirSlash = fullPath.find_last_of('/');
        if (dirSlash != std::string::npos) {
            executablePath_ = fullPath.substr(0, dirSlash);
        }
    }
    
    timestamp_ = getUTCTimestamp();
    
    // Automatically add --ip argument for camera IP address
    addArgument(CustomArg("--ip=", &g_cameraipaddress, "Specify the camera IP address"));
}

void TestRunner::addArgument(const CustomArg& arg) {
    customArgs_.push_back(arg);
}

void TestRunner::setPreTestValidator(std::function<bool()> validator) {
    preTestValidator_ = validator;
}

void TestRunner::setStrictArguments(bool strict) {
    strictArgs_ = strict;
}

int TestRunner::initialize(int& argc, char** argv) {
    // Parse custom arguments
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        
        // Check for help
        if (arg == "--help" || arg == "-h") {
            helpRequested_ = true;
            continue;
        }
        
        // Check against custom arguments
        bool matched = false;
        for (auto& customArg : customArgs_) {
            if (arg.find(customArg.prefix) == 0) {
                matched = true;
                std::string value = arg.substr(customArg.prefix.length());
                
                switch (customArg.type) {
                    case CustomArg::STRING:
                        if (customArg.targetString) {
                            *customArg.targetString = value;
                        }
                        break;
                    case CustomArg::UINT16:
                        if (customArg.targetUint16) {
                            *customArg.targetUint16 = static_cast<uint16_t>(std::stoi(value));
                        }
                        break;
                    case CustomArg::BOOL:
                        if (customArg.targetBool) {
                            *customArg.targetBool = true;
                        }
                        break;
                }
                break;
            }
        }
        
        // Handle unknown arguments
        if (!matched && strictArgs_ && !arg.empty() && arg[0] == '-') {
            // Check if it's a gtest argument
            if (arg.find("--gtest_") != 0) {
                std::cout << "Unknown argument: " << arg << std::endl;
                helpRequested_ = true;
            }
        }
    }
    
    // Show help if requested
    if (helpRequested_) {
        std::cout << "Usage: " << programName_;
        for (const auto& arg : customArgs_) {
            std::cout << " [" << arg.prefix;
            if (arg.type == CustomArg::STRING) {
                std::cout << "<value>";
            } else if (arg.type == CustomArg::UINT16) {
                std::cout << "<number>";
            }
            std::cout << "]";
        }
        std::cout << " [--help|-h]" << std::endl;
        
        for (const auto& arg : customArgs_) {
            std::cout << "  " << arg.prefix << ": " << arg.description << std::endl;
        }
        std::cout << std::endl;
        
        return 0;
    }
    
    // Create GTest output argument
    gtestOutput_ = "--gtest_output=json:report_" + execName_ + "_" + timestamp_ + ".json";
    
    // Build new argv with the additional argument
    newArgv_.clear();
    for (int i = 0; i < argc; ++i) {
        newArgv_.push_back(argv[i]);
    }
    newArgv_.push_back(const_cast<char*>(gtestOutput_.c_str()));
    newArgv_.push_back(nullptr);
    
    newArgc_ = argc + 1;
    
    // Initialize GTest
    ::testing::InitGoogleTest(&newArgc_, newArgv_.data());
    
    // Record properties
    for (const auto& customArg : customArgs_) {
        std::string propName = "Parameter " + customArg.prefix.substr(2, customArg.prefix.length() - 3);
        
        if (customArg.type == CustomArg::STRING && customArg.targetString) {
            ::testing::Test::RecordProperty(propName, *customArg.targetString);
        } else if (customArg.type == CustomArg::UINT16 && customArg.targetUint16) {
            ::testing::Test::RecordProperty(propName, *customArg.targetUint16);
        } else if (customArg.type == CustomArg::BOOL && customArg.targetBool) {
            ::testing::Test::RecordProperty(propName, *customArg.targetBool);
        }
    }
    
    return -1; // Continue to run tests
}

int TestRunner::runTests() {
    // Run pre-test validator if set
    if (preTestValidator_) {
        if (!preTestValidator_()) {
            return 1; // Validation failed
        }
    }
    
    return RUN_ALL_TESTS();
}

} // namespace aditof_test
