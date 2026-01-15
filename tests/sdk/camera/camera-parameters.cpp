#include <gtest/gtest.h>
#include <aditof/version.h>
#include <aditof/system.h>
#include <chrono>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <vector>
#include <map>
#include <functional>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/camera_definitions.h>
#include <aditof/status_definitions.h>
#include <aditof/depth_sensor_interface.h>
#include <array>
#include <cmath>
#include <thread>
#include <json.h>

using namespace aditof;

std::string g_cameraipaddress = "";

const std::vector<std::string> g_depthparams = {
    "bitsInPhaseOrDepth",
    "bitsInAB",
    "bitsInConf",
    "confThresh",
    "depthComputeIspEnable",
    "interleavingEnable",
    "jblfABThreshold",
    "jblfApplyFlag",
    "jblfExponentialTerm",
    "jblfGaussianSigma",
    "jblfMaxEdge",
    "jblfWindowSize",
    "partialDepthEnable",
    "phaseInvalid",
    "radialThreshMax",
    "radialThreshMin"
};

const std::vector<std::string> g_configurationparams = {
    "fps",
    "xyzEnable"
};

std::string g_module = "crosby";
const std::map<std::string, std::string> g_moduleJSONMap = {
    {"crosby", "depth_params_crosby.json"},
    {"tembinv2", "depth_params_tembinv2.json"},
    {"mystic", "depth_params_mystic.json"}
};

// Function to compare two JSON files
// Parameters:
//   jsonFile1: Path to the first JSON file
//   jsonFile2: Path to the second JSON file
//   differences: Map to store differences found (key -> pair of values from file1 and file2)
// Returns: true if files are identical, false if differences are found
bool compareJsonFiles(const std::string& jsonFile1,
                     const std::string& jsonFile2,
                     std::map<std::string, std::pair<double, double>>& differences) {
    
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
    std::function<void(struct json_object*, struct json_object*, const std::string&)> 
    compareObjects = [&](struct json_object* obj1, struct json_object* obj2, const std::string& path) {
        if (!obj1 || !obj2) return;

        json_object_iter iter;
        json_object_object_foreach(obj1, key, val) {
            std::string newPath = path.empty() ? key : path + "->" + key;
            
            struct json_object *val2 = nullptr;
            if (!json_object_object_get_ex(obj2, key, &val2)) {
                std::cerr << "Key '" << newPath << "' not found in second file" << std::endl;
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
                    std::cout << "Difference found at '" << newPath << "': " 
                              << val1_double << " vs " << val2_double << std::endl;
                    differences[newPath] = {val1_double, val2_double};
                }
            }
            // For other types (strings, arrays, etc.), convert to string and compare
            else {
                const char* str1 = json_object_to_json_string(val);
                const char* str2 = json_object_to_json_string(val2);
                if (std::string(str1) != std::string(str2)) {
                    std::cout << "Difference found at '" << newPath << "'" << std::endl;
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

// Function to change multiple parameters in a JSON file
// Parameters:
//   jsonFilePath: Path to the JSON file
//   sectionKey: The numbered key (e.g., "0", "1", "3")
//   subsectionKey: The subsection name (e.g., "depth-compute", "configuration-parameters")
//   parameters: Map of parameter names to their new values
// Returns: true on success, false on failure
bool changeJsonParameter(const std::string& jsonFilePath, 
                        const std::string& sectionKey,
                        const std::string& subsectionKey,
                        const std::map<std::string, double>& parameters) {
    
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
    if (!json_object_object_get_ex(section, subsectionKey.c_str(), &subsection)) {
        json_object_put(root);
        return false;
    }

    // Update all parameters
    for (const auto& param : parameters) {
        // Create a new double object with the value
        struct json_object *newValue = json_object_new_double(param.second);
        
        // Update the parameter
        json_object_object_add(subsection, param.first.c_str(), newValue);
    }

    // Write the modified JSON back to file
    if (json_object_to_file_ext(jsonFilePath.c_str(), root, JSON_C_TO_STRING_PRETTY) < 0) {
        json_object_put(root);
        return false;
    }

    json_object_put(root);
    return true;
}

// Function to read multiple parameters from a JSON file
// Parameters:
//   jsonFilePath: Path to the JSON file
//   sectionKey: The numbered key (e.g., "0", "1", "3")
//   subsectionKey: The subsection name (e.g., "depth-compute", "configuration-parameters")
//   parameterKeys: Vector of parameter names to read
//   values: Map to store the read parameter name-value pairs
// Returns: true on success (all parameters found), false on failure
bool readJsonParameter(const std::string& jsonFilePath,
                      const std::string& sectionKey,
                      const std::string& subsectionKey,
                      const std::vector<std::string>& parameterKeys,
                      std::map<std::string, double>& values) {
    
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
    if (!json_object_object_get_ex(section, subsectionKey.c_str(), &subsection)) {
        json_object_put(root);
        return false;
    }

    // Read all parameters
    bool allFound = true;
    for (const auto& paramKey : parameterKeys) {
        struct json_object *paramObj = nullptr;
        if (!json_object_object_get_ex(subsection, paramKey.c_str(), &paramObj)) {
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

// Generate UTC timestamp in format: YYYYMMDD_HHMMSS
std::string getUTCTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time_t_now = std::chrono::system_clock::to_time_t(now);
    
    std::ostringstream oss;
    oss << std::put_time(std::gmtime(&time_t_now), "%Y%m%d_%H%M%S");
    return oss.str();
}

// Test System class
TEST(SystemTest, SystemInstantiation) {
    EXPECT_NO_THROW({
        System system;
    });
}

TEST(SystemTest, GetCameraListWithoutCameras) {
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    
    // This should not throw even if no cameras are connected
    EXPECT_NO_THROW({
        if (g_cameraipaddress == "") {
            system.getCameraList(cameras);
        } else {
            system.getCameraList(cameras, "ip:" + g_cameraipaddress);
        }
    });
    
    // The cameras vector might be empty if no hardware is connected
    // This is expected in a test environment
}

bool g_callbackInvoked = false;
// Test fixture for Camera-related tests
class CameraTestFixture : public ::testing::Test {
protected:
    void SetUp() override {
        system = std::make_unique<System>();
        
        // Try to get cameras, but don't fail if none are available
        if (g_cameraipaddress == "") {
            system->getCameraList(cameras);
        } else {
            system->getCameraList(cameras, "ip:" + g_cameraipaddress);
        }
        
        if (!cameras.empty()) {
            camera = cameras.front();
            has_camera = true;

            g_callbackInvoked = false;
            // Registering a callback to be executed when ADSD3500 issues an interrupt
            std::shared_ptr<DepthSensorInterface> sensor = camera->getSensor();
            callback = [this](Adsd3500Status status) {
                g_callbackInvoked = true;
                EXPECT_EQ(status, Adsd3500Status::OK);
            };
            
            Status registerCbStatus =
                sensor->adsd3500_register_interrupt_callback(callback);
            ASSERT_TRUE(registerCbStatus == Status::OK);
        } else {
            has_camera = false;
        }
    }

    void TearDown() override {
        camera = cameras.front();
        std::shared_ptr<DepthSensorInterface> sensor = camera->getSensor();
        sensor->adsd3500_unregister_interrupt_callback(callback);
        // No explicit cleanup needed - cameras will be cleaned up automatically
        cameras.clear();
        camera.reset();
        std::remove(jsonFilePath.c_str());
    }

    std::unique_ptr<System> system;
    std::vector<std::shared_ptr<Camera>> cameras;
    std::shared_ptr<Camera> camera;
    bool has_camera = false;
    aditof::SensorInterruptCallback callback;
    const std::string jsonFilePath = "depth_params.json";
};

TEST_F(CameraTestFixture, SystemHasCameraListMethod) {
    EXPECT_NE(system, nullptr);
    // This test passes if we can call getCameraList without crashing
}

TEST_F(CameraTestFixture, CameraDetailsAccessible) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }
    
    CameraDetails details;
    Status status = camera->getDetails(details);
    
    // If we have a camera, we should be able to get its details
    if (status == Status::OK) {
        EXPECT_FALSE(details.cameraId.empty());
    }
}

TEST_F(CameraTestFixture, adsd3500SoftReset) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    bool result;
    Status init_status = camera->initialize();
    
    if (init_status == Status::OK) {

        std::remove(jsonFilePath.c_str());
        Status status = camera->saveDepthParamsToJsonFile(jsonFilePath);
        ASSERT_TRUE(status == Status::OK);

        std::vector<uint8_t> available_modes;
        status = camera->getAvailableModes(available_modes);
        ASSERT_TRUE(status == Status::OK);
        ASSERT_FALSE(available_modes.empty());

        // Compare JSON file with reference
         std::map<std::string, std::pair<double, double>> differences;
        result = compareJsonFiles(g_moduleJSONMap.at(g_module), jsonFilePath, differences);
        ASSERT_TRUE(result == true);

        // Log each parameter
        std::map<std::string, double> values;
        for (const auto& mode : available_modes) {

            values.clear();
            result = readJsonParameter(jsonFilePath,
                        std::to_string(mode),
                        "depth-compute",
                        g_depthparams,
                        values);
            ASSERT_TRUE(result);

            for (const auto &key : values) {
                RecordProperty("depth-compute_" + std::to_string(mode) + "_" + key.first, values[key.first]);
            }

            values.clear();
            result = readJsonParameter(jsonFilePath,
                        std::to_string(mode),
                        "configuration-parameters",
                        g_configurationparams,
                        values);
            ASSERT_TRUE(result);

            for (const auto &key : values) {
                RecordProperty("configuration-parameters_" + std::to_string(mode) + "_" + key.first, values[key.first]);
            }
        }
    
    } else {
        GTEST_SKIP() << "Camera initialization failed, skipping test";
    }
}

int main(int argc, char** argv) {

    bool bHelp = false;
    // Parse custom command-line argument for expected version
    for (int i = 1; i < argc; ++i) {
        std::string arg(argv[i]);
        if (arg.find("--module=") == 0) {
            g_module = arg.substr(9);  // Extract module after "--module="
        } else if (arg.find("--ip=") == 0) {
            g_cameraipaddress = arg.substr(5);  // Extract IP address after "--ip="
        }  else if (arg == "--help" || arg == "-h") {
            bHelp = true;
        } else {
            std::cout << "Unknown argument: " << arg << std::endl;
            bHelp = true;
        }
    }

    // Automatically add --gtest_output with timestamped filename
    std::string timestamp = getUTCTimestamp();
    std::string execName = argv[0];
    // Extract just the executable name without path
    size_t lastSlash = execName.find_last_of("/\\");
    if (lastSlash != std::string::npos) {
        execName = execName.substr(lastSlash + 1);
    }
    std::string gtestOutput = "--gtest_output=json:report_" + execName + "_" + timestamp + ".json";
    
    // Create new argv with the additional argument
    std::vector<char*> newArgv;
    for (int i = 0; i < argc; ++i) {
        newArgv.push_back(argv[i]);
    }
    newArgv.push_back(const_cast<char*>(gtestOutput.c_str()));
    newArgv.push_back(nullptr);
    
    int newArgc = argc + 1;

    if (bHelp) {
        std::cout << "Usage: " << argv[0] << " [--module=<module_name>] [--ip=<camera_ip_address>] [--help|-h]" << std::endl;
        std::cout << "  --module: Specify the camera module (default: crosby)" << std::endl;
        std::cout << "  --ip: Specify the camera IP address" << std::endl;
        std::cout << std::endl;
    }
    ::testing::InitGoogleTest(&newArgc, newArgv.data());
    if (bHelp) {
        return 0;
    }
    ::testing::Test::RecordProperty("Parameter module", g_module);
    ::testing::Test::RecordProperty("Parameter IP Address", g_cameraipaddress);

    for (const auto& module : g_moduleJSONMap) {
        if (module.first == g_module) {
            return RUN_ALL_TESTS();
        }
    }
    ::testing::Test::RecordProperty("Unknown Module", g_module);
}