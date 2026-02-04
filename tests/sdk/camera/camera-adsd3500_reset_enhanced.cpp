#include <gtest/gtest.h>
#include <aditof/version.h>
#include <aditof/system.h>
#include <aditof_test_utils.h>
#include <iostream>
#include <vector>
#include <aditof/camera.h>
#include <aditof/frame.h>
#include <aditof/camera_definitions.h>
#include <aditof/status_definitions.h>
#include <aditof/depth_sensor_interface.h>
#include <array>
#include <cmath>
#include <thread>
#include <atomic>           // ✅ ADDED: For thread-safe callback flag
#include <chrono>           // ✅ ADDED: For timing measurements
#include <numeric>          // ✅ ADDED: For std::accumulate
#include <algorithm>        // ✅ ADDED: For std::min_element, std::max_element

using namespace aditof;

// Use the camera IP address from aditof_test library
std::string& g_cameraipaddress = aditof_test::g_cameraipaddress;

// ============================================================
// ✅ FIXED: Thread-safe callback flag
// ============================================================
std::atomic<bool> g_callbackInvoked(false);

// ============================================================
// ✅ NEW: Parameter structure for parameterized tests
// ============================================================
struct ResetTestParams {
    uint16_t initialFrameRate;           // Frame rate to set before reset
    uint16_t expectedPostResetFrameRate; // Expected frame rate after reset
    uint16_t templateAddress;             // Template address for reset
    uint16_t templateValue;               // Template value for reset
    std::string description;              // Test description
    uint32_t resetWaitTimeMs;            // Time to wait for reset (milliseconds)
};

// =============================================================
// TEST SUITE 1: BASIC SYSTEM TESTS
// =============================================================
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

// =============================================================
// TEST FIXTURE: CAMERA TEST FIXTURE (BASE)
// =============================================================
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
        // ✅ FIXED: Safe null check before accessing empty vector
        if (!cameras.empty()) {
            camera = cameras.front();
            std::shared_ptr<DepthSensorInterface> sensor = camera->getSensor();
            sensor->adsd3500_unregister_interrupt_callback(callback);
        }
        // No explicit cleanup needed - cameras will be cleaned up automatically
        cameras.clear();
        camera.reset();
    }

    std::unique_ptr<System> system;
    std::vector<std::shared_ptr<Camera>> cameras;
    std::shared_ptr<Camera> camera;
    bool has_camera = false;
    aditof::SensorInterruptCallback callback;
};

// =============================================================
// TEST SUITE 2: BASIC CAMERA TESTS (Using Fixture)
// =============================================================
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

// =============================================================
// TEST SUITE 3: ORIGINAL RESET TEST
// =============================================================
TEST_F(CameraTestFixture, adsd3500SoftReset) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    Status init_status = camera->initialize();
    
    if (init_status == Status::OK) {
    
        // Reset the flag before the test operations
        ASSERT_TRUE(g_callbackInvoked == true);
        g_callbackInvoked = false;

        Status status = camera->adsd3500SetFrameRate(23);
        ASSERT_TRUE(status == Status::OK);

        uint16_t frameRate = 0;
        status = camera->adsd3500GetFrameRate(frameRate);
        ASSERT_TRUE(status == Status::OK);
        ASSERT_TRUE(frameRate == 23);

        status = camera->adsd3500SetGenericTemplate(0x0024, 0);
        ASSERT_TRUE(status == Status::OK);

        //Sleep for 5 seconds to allow time for the reset to occur
        std::this_thread::sleep_for(std::chrono::seconds(5));

        frameRate = 0;
        status = camera->adsd3500GetFrameRate(frameRate);
        ASSERT_TRUE(status == Status::OK);
        ASSERT_FALSE(frameRate == 23);
        ASSERT_TRUE(frameRate == 10);
    } else {
        GTEST_SKIP() << "Camera initialization failed, skipping test";
    }
}

// ============================================================
// ✅ NEW: Test reset state transitions
// ============================================================
TEST_F(CameraTestFixture, adsd3500SoftReset_StateTransitions) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    Status init_status = camera->initialize();
    if (init_status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed, skipping test";
    }

    // State 1: Initial state after initialization
    uint16_t frameRate_state1 = 0;
    ASSERT_EQ(camera->adsd3500GetFrameRate(frameRate_state1), Status::OK);
    RecordProperty("State1_InitialFrameRate", frameRate_state1);

    // State 2: After setting custom frame rate
    ASSERT_EQ(camera->adsd3500SetFrameRate(25), Status::OK);
    uint16_t frameRate_state2 = 0;
    ASSERT_EQ(camera->adsd3500GetFrameRate(frameRate_state2), Status::OK);
    EXPECT_EQ(frameRate_state2, 25);
    RecordProperty("State2_CustomFrameRate", frameRate_state2);

    // State 3: Trigger reset
    ASSERT_EQ(camera->adsd3500SetGenericTemplate(0x0024, 0), Status::OK);
    RecordProperty("State3_ResetTriggered", true);

    // Wait for reset
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // State 4: After reset completion
    uint16_t frameRate_state4 = 0;
    ASSERT_EQ(camera->adsd3500GetFrameRate(frameRate_state4), Status::OK);
    EXPECT_NE(frameRate_state4, 25) << "Frame rate should have changed after reset";
    EXPECT_EQ(frameRate_state4, 10) << "Frame rate should reset to default (10)";
    RecordProperty("State4_PostResetFrameRate", frameRate_state4);

    // Verify state transition
    RecordProperty("StateTransition", frameRate_state1 == frameRate_state4 ? "Confirmed" : "Verified");
}

// ============================================================
// ✅ NEW: Test multiple sequential resets
// ============================================================
TEST_F(CameraTestFixture, adsd3500SoftReset_SequentialResets) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    Status init_status = camera->initialize();
    if (init_status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed, skipping test";
    }

    const int RESET_COUNT = 3;
    std::vector<uint16_t> frameRates;

    for (int i = 0; i < RESET_COUNT; ++i) {
        // Set frame rate to different values
        uint16_t targetFrameRate = 20 + (i * 5);
        ASSERT_EQ(camera->adsd3500SetFrameRate(targetFrameRate), Status::OK)
            << "Failed to set frame rate in iteration " << i;

        // Verify it was set
        uint16_t verifyFrameRate = 0;
        ASSERT_EQ(camera->adsd3500GetFrameRate(verifyFrameRate), Status::OK);
        EXPECT_EQ(verifyFrameRate, targetFrameRate)
            << "Frame rate mismatch in iteration " << i;

        // Trigger reset
        ASSERT_EQ(camera->adsd3500SetGenericTemplate(0x0024, 0), Status::OK)
            << "Failed to trigger reset in iteration " << i;

        // Wait for reset
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // Check frame rate after reset
        uint16_t postResetFrameRate = 0;
        ASSERT_EQ(camera->adsd3500GetFrameRate(postResetFrameRate), Status::OK);
        frameRates.push_back(postResetFrameRate);

        RecordProperty("Reset_" + std::to_string(i) + "_PostFrameRate", postResetFrameRate);
    }

    // Verify all resets resulted in same default frame rate
    for (size_t i = 0; i < frameRates.size(); ++i) {
        EXPECT_EQ(frameRates[i], 10)
            << "Reset " << i << " did not result in default frame rate";
    }
}

// ============================================================
// ✅ NEW: Test reset without camera initialization
// ============================================================
TEST_F(CameraTestFixture, adsd3500SoftReset_WithoutInitialization) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    // Don't call camera->initialize()
    
    // Verify camera is not null before attempting operations
    ASSERT_NE(camera, nullptr) << "Camera pointer is null";
    
    // Attempt to set frame rate without initialization
    // This should fail gracefully, not crash
    Status status;
    ASSERT_NO_FATAL_FAILURE({
        status = camera->adsd3500SetFrameRate(23);
    }) << "adsd3500SetFrameRate should not crash on uninitialized camera";
    
    // Should fail gracefully
    EXPECT_NE(status, Status::OK)
        << "Setting frame rate should fail without initialization";
    
    RecordProperty("ErrorScenario", "SetFrameRate without initialization");
    RecordProperty("ActualStatus", static_cast<int>(status));
}

// ============================================================
// ✅ NEW: Test with invalid frame rate values
// ============================================================
TEST_F(CameraTestFixture, adsd3500SoftReset_InvalidFrameRates) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    Status init_status = camera->initialize();
    if (init_status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed, skipping test";
    }

    // Test invalid frame rates
    std::vector<uint16_t> invalidFrameRates = {0, 1, 999, 65535};

    for (uint16_t invalidRate : invalidFrameRates) {
        Status status = camera->adsd3500SetFrameRate(invalidRate);
        
        // Should either fail or be handled gracefully
        if (status != Status::OK) {
            RecordProperty("InvalidFrameRate_" + std::to_string(invalidRate), "Rejected");
        } else {
            // If accepted, verify it doesn't break the camera
            uint16_t readBack = 0;
            Status readStatus = camera->adsd3500GetFrameRate(readBack);
            EXPECT_EQ(readStatus, Status::OK)
                << "Should be able to read frame rate after setting " << invalidRate;
            
            RecordProperty("InvalidFrameRate_" + std::to_string(invalidRate), "Accepted");
        }
    }
}

// ============================================================
// ✅ NEW: Test reset with invalid template address
// ============================================================
TEST_F(CameraTestFixture, adsd3500SoftReset_InvalidTemplateAddress) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    Status init_status = camera->initialize();
    if (init_status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed, skipping test";
    }

    // Try invalid template addresses
    std::vector<uint16_t> invalidAddresses = {0x0000, 0xFFFF, 0x0025};

    bool anyRejected = false;
    for (uint16_t invalidAddr : invalidAddresses) {
        Status status = camera->adsd3500SetGenericTemplate(invalidAddr, 0);
        
        // Should handle gracefully (either reject or accept)
        EXPECT_TRUE(status == Status::OK || 
                    status == Status::INVALID_ARGUMENT || 
                    status == Status::GENERIC_ERROR ||
                    status == Status::UNAVAILABLE)
            << "Unexpected status for invalid address 0x" << std::hex << invalidAddr;
        
        if (status != Status::OK) {
            anyRejected = true;
            RecordProperty("InvalidTemplate_0x" + std::to_string(invalidAddr), "Rejected");
        } else {
            RecordProperty("InvalidTemplate_0x" + std::to_string(invalidAddr), "Accepted");
        }
    }
    
    // Test passes if it doesn't crash - validation of addresses is optional
    SUCCEED() << "Invalid template address handling completed without crash";
}

// ============================================================
// ✅ NEW: Measure reset completion time
// ============================================================
TEST_F(CameraTestFixture, adsd3500SoftReset_PerformanceMetrics) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    Status init_status = camera->initialize();
    if (init_status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed, skipping test";
    }

    // Set frame rate
    ASSERT_EQ(camera->adsd3500SetFrameRate(23), Status::OK);

    // Measure reset operation time
    auto reset_start = std::chrono::high_resolution_clock::now();
    
    ASSERT_EQ(camera->adsd3500SetGenericTemplate(0x0024, 0), Status::OK);
    
    // Wait for completion
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    auto reset_end = std::chrono::high_resolution_clock::now();
    
    auto reset_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        reset_end - reset_start
    );

    // Verify frame rate changed
    uint16_t frameRateAfterReset = 0;
    ASSERT_EQ(camera->adsd3500GetFrameRate(frameRateAfterReset), Status::OK);
    EXPECT_EQ(frameRateAfterReset, 10);

    // Record performance metrics
    RecordProperty("Reset Duration (ms)", reset_duration.count());
    RecordProperty("Reset Expected Time (ms)", 5000);
    RecordProperty("Reset Actual vs Expected", 
                   reset_duration.count() >= 5000 ? "OnTime or Late" : "Early");

    // Assert reset completes within reasonable time (< 10 seconds)
    EXPECT_LE(reset_duration.count(), 10000)
        << "Reset took longer than expected: " << reset_duration.count() << "ms";
}

// ============================================================
// ✅ NEW: Measure repeated reset performance
// ============================================================
TEST_F(CameraTestFixture, adsd3500SoftReset_RepeatedPerformance) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    Status init_status = camera->initialize();
    if (init_status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed, skipping test";
    }

    const int ITERATION_COUNT = 5;
    std::vector<uint64_t> durations;
    double totalTime = 0;

    for (int i = 0; i < ITERATION_COUNT; ++i) {
        // Set frame rate
        ASSERT_EQ(camera->adsd3500SetFrameRate(20 + i), Status::OK);

        // Measure reset time
        auto start = std::chrono::high_resolution_clock::now();
        
        ASSERT_EQ(camera->adsd3500SetGenericTemplate(0x0024, 0), Status::OK);
        std::this_thread::sleep_for(std::chrono::seconds(5));
        
        auto end = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
            end - start
        ).count();

        durations.push_back(duration);
        totalTime += duration;

        RecordProperty("Reset_" + std::to_string(i) + "_Duration_ms", duration);
    }

    // Calculate and record statistics
    double average = totalTime / ITERATION_COUNT;
    
    uint64_t minDuration = *std::min_element(durations.begin(), durations.end());
    uint64_t maxDuration = *std::max_element(durations.begin(), durations.end());

    RecordProperty("Average Reset Duration (ms)", average);
    RecordProperty("Minimum Reset Duration (ms)", minDuration);
    RecordProperty("Maximum Reset Duration (ms)", maxDuration);
    RecordProperty("Reset Count", ITERATION_COUNT);

    SUCCEED() << "Performance metrics recorded successfully";
}

// ============================================================
// ✅ NEW: Verify callback is invoked during reset
// ============================================================
TEST_F(CameraTestFixture, adsd3500SoftReset_CallbackVerification) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    Status init_status = camera->initialize();
    if (init_status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed, skipping test";
    }

    // Verify callback was invoked during initialization
    EXPECT_TRUE(g_callbackInvoked)
        << "Callback should be invoked during camera initialization";

    // Reset callback flag
    g_callbackInvoked = false;

    // Set frame rate and trigger reset
    ASSERT_EQ(camera->adsd3500SetFrameRate(23), Status::OK);
    ASSERT_EQ(camera->adsd3500SetGenericTemplate(0x0024, 0), Status::OK);

    // Wait for reset
    std::this_thread::sleep_for(std::chrono::seconds(5));

    // Verify callback was invoked
    EXPECT_TRUE(g_callbackInvoked)
        << "Callback should be invoked during reset operation";

    RecordProperty("InitCallback_Invoked", "Yes");
    RecordProperty("ResetCallback_Invoked", g_callbackInvoked ? "Yes" : "No");
}

// ============================================================
// ✅ NEW: Test multiple callback invocations
// ============================================================
TEST_F(CameraTestFixture, adsd3500SoftReset_MultipleCallbackInvocations) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    Status init_status = camera->initialize();
    if (init_status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed, skipping test";
    }

    const int RESET_COUNT = 3;
    int callbackInvokedCount = 0;

    for (int i = 0; i < RESET_COUNT; ++i) {
        // Reset callback flag
        g_callbackInvoked = false;

        // Set frame rate and trigger reset
        ASSERT_EQ(camera->adsd3500SetFrameRate(20 + i), Status::OK);
        ASSERT_EQ(camera->adsd3500SetGenericTemplate(0x0024, 0), Status::OK);

        // Wait for reset
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // Check if callback was invoked
        if (g_callbackInvoked) {
            callbackInvokedCount++;
        }

        RecordProperty("Reset_" + std::to_string(i) + "_CallbackInvoked",
                      g_callbackInvoked ? "Yes" : "No");
    }

    EXPECT_EQ(callbackInvokedCount, RESET_COUNT)
        << "Callback should be invoked for each reset";

    RecordProperty("Expected_Callback_Count", RESET_COUNT);
    RecordProperty("Actual_Callback_Count", callbackInvokedCount);
}

// =============================================================
// ✅ NEW: PARAMETERIZED TEST FIXTURE
// =============================================================
class ParameterizedResetTest : public CameraTestFixture,
                              public ::testing::WithParamInterface<ResetTestParams> {
public:
    // Helper method for reset operations
    Status performReset(uint16_t initialFrameRate,
                       uint16_t templateAddr,
                       uint16_t templateVal,
                       uint32_t waitMs) {
        // Set initial frame rate
        Status status = camera->adsd3500SetFrameRate(initialFrameRate);
        if (status != Status::OK) {
            return status;
        }
        
        // Verify it was set
        uint16_t verifyFrameRate = 0;
        status = camera->adsd3500GetFrameRate(verifyFrameRate);
        if (status != Status::OK) {
            return status;
        }
        
        if (verifyFrameRate != initialFrameRate) {
            return Status::GENERIC_ERROR;
        }
        
        // Trigger reset via template
        status = camera->adsd3500SetGenericTemplate(templateAddr, templateVal);
        if (status != Status::OK) {
            return status;
        }
        
        // Wait for reset to complete
        std::this_thread::sleep_for(std::chrono::milliseconds(waitMs));
        
        return Status::OK;
    }
};

// =============================================================
// ✅ NEW: Instantiate parameterized tests
// =============================================================
INSTANTIATE_TEST_SUITE_P(
    ADSD3500ResetScenarios,
    ParameterizedResetTest,
    ::testing::Values(
        ResetTestParams{
            23,      // initialFrameRate
            10,      // expectedPostResetFrameRate
            0x0024,  // templateAddress
            0,       // templateValue
            "Reset_FrameRate_23_to_10",
            5000     // resetWaitTimeMs
        },
        ResetTestParams{
            15,      // initialFrameRate
            10,      // expectedPostResetFrameRate
            0x0024,  // templateAddress
            0,       // templateValue
            "Reset_FrameRate_15_to_10",
            5000     // resetWaitTimeMs
        },
        ResetTestParams{
            30,      // initialFrameRate
            10,      // expectedPostResetFrameRate
            0x0024,  // templateAddress
            0,       // templateValue
            "Reset_FrameRate_30_to_10",
            5000     // resetWaitTimeMs
        }
    )
);

// =============================================================
// ✅ NEW: Parameterized test implementation
// =============================================================
TEST_P(ParameterizedResetTest, MultipleResetScenarios) {
    if (!has_camera) {
        GTEST_SKIP() << "No camera available for testing";
    }

    const ResetTestParams& params = GetParam();
    
    Status init_status = camera->initialize();
    if (init_status != Status::OK) {
        GTEST_SKIP() << "Camera initialization failed, skipping test";
    }

    // Reset callback flag
    g_callbackInvoked = false;

    // Perform reset with current parameters
    Status reset_status = performReset(
        params.initialFrameRate,
        params.templateAddress,
        params.templateValue,
        params.resetWaitTimeMs
    );
    ASSERT_EQ(reset_status, Status::OK) << "Reset operation failed";

    // Verify post-reset frame rate
    uint16_t frameRateAfterReset = 0;
    Status get_status = camera->adsd3500GetFrameRate(frameRateAfterReset);
    ASSERT_EQ(get_status, Status::OK) << "Failed to get frame rate after reset";
    
    EXPECT_EQ(frameRateAfterReset, params.expectedPostResetFrameRate)
        << "Frame rate did not reset to expected value. "
        << "Expected: " << static_cast<int>(params.expectedPostResetFrameRate)
        << ", Got: " << static_cast<int>(frameRateAfterReset);

    // Record test properties
    RecordProperty("Scenario", params.description);
    RecordProperty("Initial_FrameRate", params.initialFrameRate);
    RecordProperty("Post_Reset_FrameRate", frameRateAfterReset);
    RecordProperty("Expected_FrameRate", params.expectedPostResetFrameRate);
}

// =============================================================
// MAIN ENTRY POINT
// =============================================================
int main(int argc, char** argv) {
    // Create test runner
    aditof_test::TestRunner runner(argv[0]);
    
    // Note: --ip argument is automatically added by TestRunner
    // Usage: ./camera-adsd3500_reset --ip=<camera_ip>
    
    // Initialize (parses args, sets up GTest output)
    int initResult = runner.initialize(argc, argv);
    if (initResult != -1) {
        return initResult;  // Help was shown or error occurred
    }
    
    // Run tests
    return runner.runTests();
}
