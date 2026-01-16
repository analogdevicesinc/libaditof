#include <gtest/gtest.h>
#include <aditof/version.h>
#include <aditof/system.h>
#include <aditof_test_utils.h>
#include <iostream>
#include <vector>
#include <map>
#include <functional>
#include <cstdlib>
#include <climits>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/camera_definitions.h>
#include <aditof/status_definitions.h>
#include <aditof/depth_sensor_interface.h>
#include <array>
#include <cmath>
#include <thread>
#include <json.h>
#include <cstdlib>
#include <limits.h>

using namespace aditof;

// Use the camera IP address from aditof_test library
std::string& g_cameraipaddress = aditof_test::g_cameraipaddress;

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
std::map<std::string, std::string> g_moduleJSONMap = {
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
                    differences[newPath] = {val1_double, val2_double};
                }
            }
            // For other types (strings, arrays, etc.), convert to string and compare
            else {
                const char* str1 = json_object_to_json_string(val);
                const char* str2 = json_object_to_json_string(val2);
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
        std::remove(jsonFilePathWorking.c_str());
        std::remove(jsonFilePathReference.c_str());
    }

    std::unique_ptr<System> system;
    std::vector<std::shared_ptr<Camera>> cameras;
    std::shared_ptr<Camera> camera;
    bool has_camera = false;
    aditof::SensorInterruptCallback callback;
    const std::string jsonFilePathWorking = "/tmp/depth_params_working.json";
    const std::string jsonFilePathReference = "/tmp/depth_params_reference.json";
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

TEST_F(CameraTestFixture, parametercomparedefault) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    bool result;
    Status init_status = camera->initialize();
    
    if (init_status == Status::OK) {

        std::remove(jsonFilePathWorking.c_str());
        Status status = camera->saveDepthParamsToJsonFile(jsonFilePathWorking);
        ASSERT_TRUE(status == Status::OK);

        std::vector<uint8_t> available_modes;
        status = camera->getAvailableModes(available_modes);
        ASSERT_TRUE(status == Status::OK);
        ASSERT_FALSE(available_modes.empty());

        // Compare JSON file with reference
         std::map<std::string, std::pair<double, double>> differences;
        result = compareJsonFiles(g_moduleJSONMap.at(g_module), jsonFilePathWorking, differences);
        // Print differences if any
        for (const auto& diff : differences) {
            RecordProperty("Difference at '" + diff.first + "'",
                              std::to_string(diff.second.first) + " vs " + std::to_string(diff.second.second));
            std::cout << "Difference at '" << diff.first << "': " 
                      << diff.second.first << " vs " << diff.second.second << std::endl;
        }
        ASSERT_TRUE(result == true);
    
    } else {
        GTEST_SKIP() << "Camera initialization failed, skipping test";
    }
}

TEST_F(CameraTestFixture, parameterchangevalues) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    bool result;
    Status init_status = camera->initialize();
    
    if (init_status == Status::OK) {

        std::remove(jsonFilePathWorking.c_str());
        Status status = camera->saveDepthParamsToJsonFile(jsonFilePathWorking);
        ASSERT_TRUE(status == Status::OK);

        status = camera->saveDepthParamsToJsonFile(jsonFilePathReference);
        ASSERT_TRUE(status == Status::OK);

        std::vector<uint8_t> available_modes;
        status = camera->getAvailableModes(available_modes);
        ASSERT_TRUE(status == Status::OK);
        ASSERT_FALSE(available_modes.empty());

        // Log each parameter
        std::map<std::string, double> values;
        for (const auto& mode : available_modes) {

            values.clear();
            result = readJsonParameter(jsonFilePathWorking,
                        std::to_string(mode),
                        "depth-compute",
                        g_depthparams,
                        values);
            ASSERT_TRUE(result);
            // Modify each parameter by adding 1.0
            for (auto& param : values) {
                param.second += 1.0;
            }
            result = changeJsonParameter(jsonFilePathWorking,
                        std::to_string(mode),
                        "depth-compute",
                        values);
            ASSERT_TRUE(result);

            values.clear();
            result = readJsonParameter(jsonFilePathWorking,
                        std::to_string(mode),
                        "configuration-parameters",
                        g_configurationparams,
                        values);
            ASSERT_TRUE(result);
            // Modify each parameter by adding 1.0
            for (auto& param : values) {
                param.second += 1.0;
            }
            result = changeJsonParameter(jsonFilePathWorking,
                        std::to_string(mode),
                        "configuration-parameters",
                        values);
            ASSERT_TRUE(result);
        }

        // Make sure changed values are not lingering in the system
        status = camera->loadDepthParamsFromJsonFile(jsonFilePathReference);
        ASSERT_TRUE(status == Status::OK);

        // Load modified parameters
        status = camera->loadDepthParamsFromJsonFile(jsonFilePathWorking);
        ASSERT_TRUE(status == Status::OK);

        std::map<std::string, std::pair<double, double>> differences;
        compareJsonFiles(jsonFilePathWorking, jsonFilePathReference, differences);
        ASSERT_TRUE(result == true);
        // Print differences if any
        for (const auto& diff : differences) {
            if (diff.second.first - 1 != diff.second.second) {
                RecordProperty("Difference at '" + diff.first + "'",
                                std::to_string(diff.second.first) + " vs " + std::to_string(diff.second.second));
                std::cout << "Difference at '" << diff.first << "': " 
                        << diff.second.first << " vs " << diff.second.second << std::endl;
            }
        }
    
    } else {
        GTEST_SKIP() << "Camera initialization failed, skipping test";
    }
}

int main(int argc, char** argv) {
    // Create test runner
    aditof_test::TestRunner runner(argv[0]);
    
    // Add custom arguments
    runner.addArgument({"--module=", &g_module, "Specify the camera module (default: crosby)"});
    // Note: --ip argument is automatically added by TestRunner
    
    // Initialize (parses args, sets up GTest output)
    int initResult = runner.initialize(argc, argv);
    if (initResult != -1) {
        return initResult;  // Help was shown or error occurred
    }
    
    // Set pre-test validator to check module is valid
    runner.setPreTestValidator([]() {
        for (const auto& module : g_moduleJSONMap) {
            if (module.first == g_module) {
                return true;
            }
        }
        ::testing::Test::RecordProperty("Unknown Module", g_module);
        return false;
    });

    std::string fullPath = runner.getExecutablePath();
    for (const auto& module : g_moduleJSONMap) {
        g_moduleJSONMap.at(module.first) =  fullPath + "/" + g_moduleJSONMap.at(module.first);
    }
    
    // Run tests
    return runner.runTests();
}