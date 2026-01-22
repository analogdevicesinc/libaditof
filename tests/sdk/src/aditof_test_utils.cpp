#include "aditof_test_utils.h"
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <functional>
#include <iomanip>
#include <iostream>
#include <json.h>
#include <sstream>

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

TestRunner::TestRunner(const std::string &programName)
    : programName_(programName), execName_(programName), helpRequested_(false),
      strictArgs_(true), preTestValidator_(nullptr), newArgc_(0),
      executablePath_(programName) {

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
    addArgument(CustomArg("--ip=", &g_cameraipaddress,
                          "Specify the camera IP address"));
}

void TestRunner::addArgument(const CustomArg &arg) {
    customArgs_.push_back(arg);
}

void TestRunner::setPreTestValidator(std::function<bool()> validator) {
    preTestValidator_ = validator;
}

void TestRunner::setStrictArguments(bool strict) { strictArgs_ = strict; }

int TestRunner::initialize(int &argc, char **argv) {
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
        for (auto &customArg : customArgs_) {
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
                        *customArg.targetUint16 =
                            static_cast<uint16_t>(std::stoi(value));
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
        for (const auto &arg : customArgs_) {
            std::cout << " [" << arg.prefix;
            if (arg.type == CustomArg::STRING) {
                std::cout << "<value>";
            } else if (arg.type == CustomArg::UINT16) {
                std::cout << "<number>";
            }
            std::cout << "]";
        }
        std::cout << " [--help|-h]" << std::endl;

        for (const auto &arg : customArgs_) {
            std::cout << "  " << arg.prefix << ": " << arg.description
                      << std::endl;
        }
        std::cout << std::endl;

        return 0;
    }

    // Create GTest output argument
    gtestOutput_ =
        "--gtest_output=json:report_" + execName_ + "_" + timestamp_ + ".json";

    // Build new argv with the additional argument
    newArgv_.clear();
    for (int i = 0; i < argc; ++i) {
        newArgv_.push_back(argv[i]);
    }
    newArgv_.push_back(const_cast<char *>(gtestOutput_.c_str()));
    newArgv_.push_back(nullptr);

    newArgc_ = argc + 1;

    // Initialize GTest
    ::testing::InitGoogleTest(&newArgc_, newArgv_.data());

    // Record properties
    for (const auto &customArg : customArgs_) {
        std::string propName =
            "Parameter " +
            customArg.prefix.substr(2, customArg.prefix.length() - 3);

        if (customArg.type == CustomArg::STRING && customArg.targetString) {
            ::testing::Test::RecordProperty(propName, *customArg.targetString);
        } else if (customArg.type == CustomArg::UINT16 &&
                   customArg.targetUint16) {
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

// JSON utility function implementations

bool compareJsonFiles(
    const std::string &jsonFile1, const std::string &jsonFile2,
    std::map<std::string, std::pair<double, double>> &differences) {

    differences.clear();

    // Parse both JSON files
    struct json_object *root1 = json_object_from_file(jsonFile1.c_str());
    if (!root1) {
        std::cerr << "Failed to parse JSON file: " << jsonFile1 << std::endl;
        return false;
    }

    struct json_object *root2 = json_object_from_file(jsonFile2.c_str());
    if (!root2) {
        std::cerr << "Failed to parse JSON file: " << jsonFile2 << std::endl;
        json_object_put(root1);
        return false;
    }

    // Recursively compare all numeric values in the JSON structures
    // Helper lambda to recursively traverse and compare
    std::function<void(struct json_object *, struct json_object *,
                       const std::string &)>
        compareObjects = [&](struct json_object *obj1, struct json_object *obj2,
                             const std::string &path) {
            if (!obj1 || !obj2)
                return;

            json_object_object_foreach(obj1, key, val) {
                std::string newPath = path.empty() ? key : path + "->" + key;

                struct json_object *val2 = nullptr;
                if (!json_object_object_get_ex(obj2, key, &val2)) {
                    std::cerr << "Key '" << newPath
                              << "' not found in second file" << std::endl;
                    differences[newPath] = {std::nan(""), std::nan("")};
                    continue;
                }

                // Check if both are objects - recurse
                if (json_object_is_type(val, json_type_object) &&
                    json_object_is_type(val2, json_type_object)) {
                    compareObjects(val, val2, newPath);
                }
                // Check if both are doubles/numbers
                else if (json_object_is_type(val, json_type_double) ||
                         json_object_is_type(val, json_type_int)) {
                    double val1_double = json_object_get_double(val);
                    double val2_double = json_object_get_double(val2);

                    // Compare with small epsilon for floating point
                    if (std::abs(val1_double - val2_double) > 1e-9) {
                        differences[newPath] = {val1_double, val2_double};
                    }
                }
                // For other types (strings, arrays, etc.), convert to string and compare
                else {
                    const char *str1 = json_object_to_json_string(val);
                    const char *str2 = json_object_to_json_string(val2);
                    if (std::string(str1) != std::string(str2)) {
                        differences[newPath] = {std::nan(""), std::nan("")};
                    }
                }
            }
        };

    compareObjects(root1, root2, "");

    bool isIdentical = differences.empty();

    json_object_put(root1);
    json_object_put(root2);
    return isIdentical;
}

bool changeJsonParameter(const std::string &jsonFilePath,
                         const std::string &sectionKey,
                         const std::string &subsectionKey,
                         const std::map<std::string, double> &parameters) {

    if (parameters.empty()) {
        return false;
    }

    // Read JSON from file
    FILE *fp = fopen(jsonFilePath.c_str(), "r");
    if (!fp) {
        return false;
    }

    // Parse JSON
    struct json_object *root = json_object_from_file(jsonFilePath.c_str());
    if (!root) {
        fclose(fp);
        return false;
    }
    fclose(fp);

    // Get the section object (e.g., "0", "1", "3")
    struct json_object *section = nullptr;
    if (!json_object_object_get_ex(root, sectionKey.c_str(), &section)) {
        json_object_put(root);
        return false;
    }

    // Get the subsection object (e.g., "depth-compute", "configuration-parameters")
    struct json_object *subsection = nullptr;
    if (!json_object_object_get_ex(section, subsectionKey.c_str(),
                                   &subsection)) {
        json_object_put(root);
        return false;
    }

    // Update all parameters
    for (const auto &param : parameters) {
        // Create a new double object with the value
        struct json_object *newValue = json_object_new_double(param.second);

        // Update the parameter
        json_object_object_add(subsection, param.first.c_str(), newValue);
    }

    // Write the modified JSON back to file
    if (json_object_to_file_ext(jsonFilePath.c_str(), root,
                                JSON_C_TO_STRING_PRETTY) < 0) {
        json_object_put(root);
        return false;
    }

    json_object_put(root);
    return true;
}

bool readJsonParameter(const std::string &jsonFilePath,
                       const std::string &sectionKey,
                       const std::string &subsectionKey,
                       const std::vector<std::string> &parameterKeys,
                       std::map<std::string, double> &values) {

    if (parameterKeys.empty()) {
        return false;
    }

    values.clear();

    // Parse JSON from file
    struct json_object *root = json_object_from_file(jsonFilePath.c_str());
    if (!root) {
        return false;
    }

    // Get the section object (e.g., "0", "1", "3")
    struct json_object *section = nullptr;
    if (!json_object_object_get_ex(root, sectionKey.c_str(), &section)) {
        json_object_put(root);
        return false;
    }

    // Get the subsection object (e.g., "depth-compute", "configuration-parameters")
    struct json_object *subsection = nullptr;
    if (!json_object_object_get_ex(section, subsectionKey.c_str(),
                                   &subsection)) {
        json_object_put(root);
        return false;
    }

    // Read all parameters
    bool allFound = true;
    for (const auto &paramKey : parameterKeys) {
        struct json_object *paramObj = nullptr;
        if (!json_object_object_get_ex(subsection, paramKey.c_str(),
                                       &paramObj)) {
            allFound = false;
            continue;
        }

        // Extract the value
        double value = json_object_get_double(paramObj);
        values[paramKey] = value;
    }

    json_object_put(root);
    return allFound;
}

} // namespace aditof_test
