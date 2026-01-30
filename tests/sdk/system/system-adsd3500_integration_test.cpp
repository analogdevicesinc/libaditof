/**
 * SYSTEM TEST: ADSD3500 Sensor Integration
 * 
 * Tests the ADSD3500 depth sensor integration:
 * - Sensor configuration (thresholds, filters)
 * - Temperature monitoring
 * - Firmware version
 * - Reset and recovery
 */

#include <gtest/gtest.h>
#include <aditof/system.h>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/camera_definitions.h>
#include <aditof/status_definitions.h>
#include <aditof_test_utils.h>
#include <vector>
#include <memory>
#include <string>

using namespace aditof;

std::string& g_cameraipaddress = aditof_test::g_cameraipaddress;

/**
 * ADSD3500IntegrationTest - Tests ADSD3500-specific functionality
 */
class ADSD3500IntegrationTest : public ::testing::Test {
protected:
    System system;
    std::vector<std::shared_ptr<Camera>> cameras;
    std::shared_ptr<Camera> camera;
    
    void SetUp() override {
        Status status;
        if (g_cameraipaddress.empty()) {
            status = system.getCameraList(cameras);
        } else {
            status = system.getCameraList(cameras, "ip:" + g_cameraipaddress);
        }
        
        if (status != Status::OK || cameras.empty()) {
            GTEST_SKIP_("No camera available for ADSD3500 test");
        }
        
        camera = cameras[0];
        status = camera->initialize();
        if (status != Status::OK) {
            GTEST_SKIP_("Camera initialization failed");
        }
    }
    
    void TearDown() override {
        if (camera) {
            camera->stop();
        }
    }
};

// Test: Get ADSD3500 firmware version
TEST_F(ADSD3500IntegrationTest, GetFirmwareVersion) {
    std::string fwVersion;
    std::string fwHash;
    
    Status status = camera->adsd3500GetFirmwareVersion(fwVersion, fwHash);
    
    if (status == Status::OK) {
        EXPECT_FALSE(fwVersion.empty()) << "Firmware version should not be empty";
        ::testing::Test::RecordProperty("FirmwareVersion", fwVersion.c_str());
        ::testing::Test::RecordProperty("FirmwareHash", fwHash.c_str());
    }
    // May return UNAVAILABLE if not supported
    EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
}

// Test: Get/Set AB invalidation threshold
TEST_F(ADSD3500IntegrationTest, ABInvalidationThreshold) {
    int originalThreshold = 0;
    Status status = camera->adsd3500GetABinvalidationThreshold(originalThreshold);
    
    if (status == Status::OK) {
        // Try setting a new threshold
        int newThreshold = 10;
        status = camera->adsd3500SetABinvalidationThreshold(newThreshold);
        EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
        
        // Read back and verify
        int readBackThreshold = 0;
        status = camera->adsd3500GetABinvalidationThreshold(readBackThreshold);
        if (status == Status::OK) {
            EXPECT_EQ(readBackThreshold, newThreshold);
        }
        
        // Restore original
        camera->adsd3500SetABinvalidationThreshold(originalThreshold);
    }
}

// Test: Get/Set confidence threshold
TEST_F(ADSD3500IntegrationTest, ConfidenceThreshold) {
    int originalThreshold = 0;
    Status status = camera->adsd3500GetConfidenceThreshold(originalThreshold);
    
    if (status == Status::OK) {
        int newThreshold = 5;
        status = camera->adsd3500SetConfidenceThreshold(newThreshold);
        EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
        
        int readBack = 0;
        status = camera->adsd3500GetConfidenceThreshold(readBack);
        if (status == Status::OK) {
            EXPECT_EQ(readBack, newThreshold);
        }
        
        // Restore
        camera->adsd3500SetConfidenceThreshold(originalThreshold);
    }
}

// Test: JBLF filter enable/disable
TEST_F(ADSD3500IntegrationTest, JBLFFilterEnableState) {
    bool originalState = false;
    Status status = camera->adsd3500GetJBLFfilterEnableState(originalState);
    
    if (status == Status::OK) {
        // Toggle state
        status = camera->adsd3500SetJBLFfilterEnableState(!originalState);
        EXPECT_TRUE(status == Status::OK || status == Status::UNAVAILABLE);
        
        // Read back
        bool newState = false;
        status = camera->adsd3500GetJBLFfilterEnableState(newState);
        if (status == Status::OK) {
            EXPECT_EQ(newState, !originalState);
        }
        
        // Restore
        camera->adsd3500SetJBLFfilterEnableState(originalState);
    }
}

// Test: JBLF filter size
TEST_F(ADSD3500IntegrationTest, JBLFFilterSize) {
    int originalSize = 0;
    Status status = camera->adsd3500GetJBLFfilterSize(originalSize);
    
    if (status == Status::OK) {
        // Valid sizes are 3, 5, 7
        std::vector<int> validSizes = {3, 5, 7};
        
        for (int size : validSizes) {
            status = camera->adsd3500SetJBLFfilterSize(size);
            if (status == Status::OK) {
                int readBack = 0;
                camera->adsd3500GetJBLFfilterSize(readBack);
                EXPECT_EQ(readBack, size);
            }
        }
        
        // Restore
        camera->adsd3500SetJBLFfilterSize(originalSize);
    }
}

// Test: Radial threshold min/max
TEST_F(ADSD3500IntegrationTest, RadialThresholds) {
    int originalMin = 0, originalMax = 0;
    
    Status statusMin = camera->adsd3500GetRadialThresholdMin(originalMin);
    Status statusMax = camera->adsd3500GetRadialThresholdMax(originalMax);
    
    if (statusMin == Status::OK && statusMax == Status::OK) {
        // Test setting new thresholds
        int newMin = 100, newMax = 5000;
        
        camera->adsd3500SetRadialThresholdMin(newMin);
        camera->adsd3500SetRadialThresholdMax(newMax);
        
        int readMin = 0, readMax = 0;
        camera->adsd3500GetRadialThresholdMin(readMin);
        camera->adsd3500GetRadialThresholdMax(readMax);
        
        // Values may be clamped by hardware
        EXPECT_GE(readMin, 0);
        EXPECT_GE(readMax, readMin);
        
        // Restore
        camera->adsd3500SetRadialThresholdMin(originalMin);
        camera->adsd3500SetRadialThresholdMax(originalMax);
    }
}

// Test: Get sensor temperature
TEST_F(ADSD3500IntegrationTest, GetSensorTemperature) {
    uint16_t temperature = 0;
    Status status = camera->adsd3500GetSensorTemperature(temperature);
    
    if (status == Status::OK) {
        // Temperature should be reasonable (0-100°C typical)
        EXPECT_LT(temperature, 150) << "Sensor temperature unreasonably high";
        ::testing::Test::RecordProperty("SensorTemperature_C", temperature);
    }
}

// Test: Get laser temperature
TEST_F(ADSD3500IntegrationTest, GetLaserTemperature) {
    uint16_t temperature = 0;
    Status status = camera->adsd3500GetLaserTemperature(temperature);
    
    if (status == Status::OK) {
        EXPECT_LT(temperature, 150) << "Laser temperature unreasonably high";
        ::testing::Test::RecordProperty("LaserTemperature_C", temperature);
    }
}

// Test: Frame capture after ADSD3500 configuration
TEST_F(ADSD3500IntegrationTest, FrameCaptureAfterConfiguration) {
    // Configure some settings
    camera->adsd3500SetConfidenceThreshold(10);
    camera->adsd3500SetJBLFfilterEnableState(true);
    
    // Set mode and start
    std::vector<uint8_t> modes;
    Status status = camera->getAvailableModes(modes);
    ASSERT_EQ(status, Status::OK);
    ASSERT_FALSE(modes.empty());
    
    status = camera->setMode(modes[0]);
    ASSERT_EQ(status, Status::OK);
    
    status = camera->start();
    ASSERT_EQ(status, Status::OK);
    
    // Capture frame after configuration
    Frame frame;
    status = camera->requestFrame(&frame);
    EXPECT_EQ(status, Status::OK) << "Frame capture failed after ADSD3500 configuration";
    
    camera->stop();
}

int main(int argc, char** argv) {
    aditof_test::TestRunner runner(argv[0]);
    
    int initResult = runner.initialize(argc, argv);
    if (initResult != -1) {
        return initResult;
    }
    
    return runner.runTests();
}
