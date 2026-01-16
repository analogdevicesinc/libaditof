#include <gtest/gtest.h>
#include <aditof/version.h>
#include <aditof/system.h>
#include <aditof_test_utils.h>
#include <iostream>
#include <vector>
#include <map>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/camera_definitions.h>
#include <aditof/status_definitions.h>
#include <aditof/depth_sensor_interface.h>
#include <array>
#include <cmath>
#include <thread>
#include <cstdlib>

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
        result = aditof_test::compareJsonFiles(g_moduleJSONMap.at(g_module), jsonFilePathWorking, differences);
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
            result = aditof_test::readJsonParameter(jsonFilePathWorking,
                        std::to_string(mode),
                        "depth-compute",
                        g_depthparams,
                        values);
            ASSERT_TRUE(result);
            // Modify each parameter by adding 1.0
            for (auto& param : values) {
                param.second += 1.0;
            }
            result = aditof_test::changeJsonParameter(jsonFilePathWorking,
                        std::to_string(mode),
                        "depth-compute",
                        values);
            ASSERT_TRUE(result);

            values.clear();
            result = aditof_test::readJsonParameter(jsonFilePathWorking,
                        std::to_string(mode),
                        "configuration-parameters",
                        g_configurationparams,
                        values);
            ASSERT_TRUE(result);
            // Modify each parameter by adding 1.0
            for (auto& param : values) {
                param.second += 1.0;
            }
            result = aditof_test::changeJsonParameter(jsonFilePathWorking,
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
        aditof_test::compareJsonFiles(jsonFilePathWorking, jsonFilePathReference, differences);
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