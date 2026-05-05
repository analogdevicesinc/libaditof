/*
 * MIT License
 *
 * Copyright (c) 2026 Analog Devices, Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <aditof/camera_definitions.h>
#include <aditof/frame_definitions.h>
#include <aditof/frame_operations.h>
#include <aditof/status_definitions.h>
#include <gtest/gtest.h>
#include <sstream>

using namespace aditof;

/**
 * @brief Unit tests for SDK type definitions (Phase 3)
 * 
 * Tests cover:
 * - Status enum values and stream output
 * - Adsd3500Status enum values and stream output
 * - FrameDetails construction and comparison
 * - FrameDataDetails structure operations
 * - CameraDetails access patterns
 * - IntrinsicParameters operations
 */

// ============================================================================
// Status Enum Tests
// ============================================================================

TEST(StatusTest, StatusOKIsZero) { EXPECT_EQ(static_cast<int>(Status::OK), 0); }

TEST(StatusTest, StatusEnumValuesAreDistinct) {
    EXPECT_NE(Status::OK, Status::BUSY);
    EXPECT_NE(Status::OK, Status::UNREACHABLE);
    EXPECT_NE(Status::OK, Status::INVALID_ARGUMENT);
    EXPECT_NE(Status::OK, Status::UNAVAILABLE);
    EXPECT_NE(Status::OK, Status::INSUFFICIENT_MEMORY);
    EXPECT_NE(Status::OK, Status::GENERIC_ERROR);
}

TEST(StatusTest, StatusStreamOutput) {
    std::ostringstream oss;

    oss << Status::OK;
    EXPECT_EQ(oss.str(), "Status::OK");
    oss.str("");

    oss << Status::BUSY;
    EXPECT_EQ(oss.str(), "Status::BUSY");
    oss.str("");

    oss << Status::UNREACHABLE;
    EXPECT_EQ(oss.str(), "Status::UNREACHABLE");
    oss.str("");

    oss << Status::INVALID_ARGUMENT;
    EXPECT_EQ(oss.str(), "Status::INVALID_ARGUMENT");
    oss.str("");

    oss << Status::UNAVAILABLE;
    EXPECT_EQ(oss.str(), "Status::UNAVAILABLE");
    oss.str("");

    oss << Status::INSUFFICIENT_MEMORY;
    EXPECT_EQ(oss.str(), "Status::INSUFFICIENT_MEMORY");
    oss.str("");

    oss << Status::GENERIC_ERROR;
    EXPECT_EQ(oss.str(), "Status::GENERIC_ERROR");
}

TEST(StatusTest, StatusComparisonWorks) {
    Status s1 = Status::OK;
    Status s2 = Status::OK;
    Status s3 = Status::GENERIC_ERROR;

    EXPECT_EQ(s1, s2);
    EXPECT_NE(s1, s3);
    EXPECT_TRUE(s1 == Status::OK);
    EXPECT_FALSE(s1 == Status::GENERIC_ERROR);
}

// ============================================================================
// Adsd3500Status Enum Tests
// ============================================================================

TEST(Adsd3500StatusTest, Adsd3500StatusOKIsZero) {
    EXPECT_EQ(static_cast<int>(Adsd3500Status::OK), 0);
}

TEST(Adsd3500StatusTest, CommonAdsd3500StatusValues) {
    // Test commonly used status values
    std::ostringstream oss;

    oss << Adsd3500Status::OK;
    EXPECT_EQ(oss.str(), "Adsd3500Status::OK");
    oss.str("");

    oss << Adsd3500Status::INVALID_MODE;
    EXPECT_EQ(oss.str(), "Adsd3500Status::INVALID_MODE");
    oss.str("");

    oss << Adsd3500Status::TIMEOUT_ERROR;
    EXPECT_EQ(oss.str(), "Adsd3500Status::TIMEOUT_ERROR");
}

TEST(Adsd3500StatusTest, FirmwareUpdateStatuses) {
    std::ostringstream oss;

    oss << Adsd3500Status::FIRMWARE_UPDATE_COMPLETE;
    EXPECT_EQ(oss.str(), "Adsd3500Status::FIRMWARE_UPDATE_COMPLETE");
    oss.str("");

    oss << Adsd3500Status::NVM_WRITE_COMPLETE;
    EXPECT_EQ(oss.str(), "Adsd3500Status::NVM_WRITE_COMPLETE");
}

TEST(Adsd3500StatusTest, ErrorStatuses) {
    std::ostringstream oss;

    oss << Adsd3500Status::IMAGER_ERROR;
    EXPECT_EQ(oss.str(), "Adsd3500Status::IMAGER_ERROR");
    oss.str("");

    oss << Adsd3500Status::INVALID_FIRMWARE_CRC;
    EXPECT_EQ(oss.str(), "Adsd3500Status::INVALID_FIRMWARE_CRC");
}

// ============================================================================
// FrameDataDetails Tests
// ============================================================================

TEST(FrameDataDetailsTest, DefaultConstruction) {
    FrameDataDetails details;
    // Should construct without error - numeric values are uninitialized
    // Just verify we can set values
    details.width = 512;
    details.height = 512;
    EXPECT_EQ(details.width, 512);
    EXPECT_EQ(details.height, 512);
    EXPECT_TRUE(details.type.empty());
}

TEST(FrameDataDetailsTest, SetAndGetProperties) {
    FrameDataDetails details;
    details.type = "depth";
    details.width = 512;
    details.height = 512;
    details.subelementSize = 2;
    details.subelementsPerElement = 1;

    EXPECT_EQ(details.type, "depth");
    EXPECT_EQ(details.width, 512);
    EXPECT_EQ(details.height, 512);
    EXPECT_EQ(details.subelementSize, 2);
    EXPECT_EQ(details.subelementsPerElement, 1);
}

TEST(FrameDataDetailsTest, CalculateBytesCount) {
    FrameDataDetails depthDetails;
    depthDetails.type = "depth";
    depthDetails.width = 512;
    depthDetails.height = 512;
    depthDetails.subelementSize = 2; // uint16_t
    depthDetails.subelementsPerElement = 1;

    // Expected bytes: 512 * 512 * 2 * 1 = 524288
    uint32_t expectedBytes = depthDetails.width * depthDetails.height *
                             depthDetails.subelementSize *
                             depthDetails.subelementsPerElement;
    EXPECT_EQ(expectedBytes, 524288);
}

TEST(FrameDataDetailsTest, ConfidenceDataTypeIsFloat) {
    FrameDataDetails confDetails;
    confDetails.type = "conf";
    confDetails.width = 512;
    confDetails.height = 512;
    confDetails.subelementSize = 4; // float
    confDetails.subelementsPerElement = 1;

    EXPECT_EQ(confDetails.subelementSize, sizeof(float));
}

TEST(FrameDataDetailsTest, XYZDataTypeHasThreeSubelements) {
    FrameDataDetails xyzDetails;
    xyzDetails.type = "xyz";
    xyzDetails.width = 512;
    xyzDetails.height = 512;
    xyzDetails.subelementSize = 2;        // uint16_t per coordinate
    xyzDetails.subelementsPerElement = 3; // x, y, z

    EXPECT_EQ(xyzDetails.subelementsPerElement, 3);

    // Expected bytes: 512 * 512 * 2 * 3 = 1572864
    uint32_t expectedBytes = xyzDetails.width * xyzDetails.height *
                             xyzDetails.subelementSize *
                             xyzDetails.subelementsPerElement;
    EXPECT_EQ(expectedBytes, 1572864);
}

// ============================================================================
// FrameDetails Tests
// ============================================================================

TEST(FrameDetailsTest, DefaultConstruction) {
    FrameDetails details;
    // Should construct without error - values are uninitialized
    // Just verify we can assign
    details.width = 0;
    details.height = 0;
    EXPECT_EQ(details.width, 0);
    EXPECT_EQ(details.height, 0);
}

TEST(FrameDetailsTest, SetBasicProperties) {
    FrameDetails details;
    details.width = 512;
    details.height = 512;
    details.type = "QMP";
    details.cameraMode = "qmp_mode";

    EXPECT_EQ(details.width, 512);
    EXPECT_EQ(details.height, 512);
    EXPECT_EQ(details.type, "QMP");
    EXPECT_EQ(details.cameraMode, "qmp_mode");
}

TEST(FrameDetailsTest, AddMultipleDataDetails) {
    FrameDetails details;
    details.width = 512;
    details.height = 512;

    FrameDataDetails depthDetails;
    depthDetails.type = "depth";
    depthDetails.width = 512;
    depthDetails.height = 512;
    depthDetails.subelementSize = 2;
    depthDetails.subelementsPerElement = 1;
    details.dataDetails.push_back(depthDetails);

    FrameDataDetails abDetails;
    abDetails.type = "ab";
    abDetails.width = 512;
    abDetails.height = 512;
    abDetails.subelementSize = 2;
    abDetails.subelementsPerElement = 1;
    details.dataDetails.push_back(abDetails);

    FrameDataDetails confDetails;
    confDetails.type = "conf";
    confDetails.width = 512;
    confDetails.height = 512;
    confDetails.subelementSize = 4;
    confDetails.subelementsPerElement = 1;
    details.dataDetails.push_back(confDetails);

    EXPECT_EQ(details.dataDetails.size(), 3);
    EXPECT_EQ(details.dataDetails[0].type, "depth");
    EXPECT_EQ(details.dataDetails[1].type, "ab");
    EXPECT_EQ(details.dataDetails[2].type, "conf");
}

TEST(FrameDetailsTest, EqualityOperator) {
    FrameDetails details1;
    details1.width = 512;
    details1.height = 512;
    details1.type = "QMP";

    FrameDetails details2;
    details2.width = 512;
    details2.height = 512;
    details2.type = "QMP";

    FrameDetails details3;
    details3.width = 1024;
    details3.height = 1024;
    details3.type = "MP";

    EXPECT_EQ(details1, details2);
    EXPECT_NE(details1, details3);
}

TEST(FrameDetailsTest, CopyConstruction) {
    FrameDetails original;
    original.width = 512;
    original.height = 512;
    original.type = "QMP";

    FrameDataDetails depthDetails;
    depthDetails.type = "depth";
    depthDetails.width = 512;
    depthDetails.height = 512;
    original.dataDetails.push_back(depthDetails);

    FrameDetails copy(original);
    EXPECT_EQ(copy.width, 512);
    EXPECT_EQ(copy.height, 512);
    EXPECT_EQ(copy.type, "QMP");
    EXPECT_EQ(copy.dataDetails.size(), 1);
    EXPECT_EQ(copy.dataDetails[0].type, "depth");
}

TEST(FrameDetailsTest, MPModeDetails) {
    FrameDetails mpDetails;
    mpDetails.width = 1024;
    mpDetails.height = 1024;
    mpDetails.type = "MP";
    mpDetails.cameraMode = "mp_mode";

    EXPECT_EQ(mpDetails.width, 1024);
    EXPECT_EQ(mpDetails.height, 1024);
    EXPECT_EQ(mpDetails.type, "MP");
}

TEST(FrameDetailsTest, QMPModeDetails) {
    FrameDetails qmpDetails;
    qmpDetails.width = 512;
    qmpDetails.height = 512;
    qmpDetails.type = "QMP";
    qmpDetails.cameraMode = "qmp_mode";

    EXPECT_EQ(qmpDetails.width, 512);
    EXPECT_EQ(qmpDetails.height, 512);
    EXPECT_EQ(qmpDetails.type, "QMP");
}

// ============================================================================
// CameraDetails Tests
// ============================================================================

TEST(CameraDetailsTest, DefaultConstruction) {
    CameraDetails details;
    // Should construct without error - connection value is uninitialized
    // Just verify we can set values
    details.connection = ConnectionType::ON_TARGET;
    EXPECT_EQ(details.connection, ConnectionType::ON_TARGET);
    EXPECT_TRUE(details.cameraId.empty());
}

TEST(CameraDetailsTest, SetProperties) {
    CameraDetails details;
    details.cameraId = "adsd3500_001";
    details.connection = ConnectionType::NETWORK;

    EXPECT_EQ(details.cameraId, "adsd3500_001");
    EXPECT_EQ(details.connection, ConnectionType::NETWORK);
}

TEST(CameraDetailsTest, SetMultipleFields) {
    CameraDetails details;
    details.cameraId = "adsd3500";
    details.connection = ConnectionType::NETWORK;
    details.mode = 2; // Mode index

    EXPECT_EQ(details.cameraId, "adsd3500");
    EXPECT_EQ(details.connection, ConnectionType::NETWORK);
    EXPECT_EQ(details.mode, 2);
}

TEST(CameraDetailsTest, ConnectionTypesAreDistinct) {
    EXPECT_NE(ConnectionType::ON_TARGET, ConnectionType::NETWORK);
    EXPECT_NE(ConnectionType::ON_TARGET, ConnectionType::OFFLINE);
    EXPECT_NE(ConnectionType::NETWORK, ConnectionType::OFFLINE);
}

// ============================================================================
// IntrinsicParameters Tests
// ============================================================================

TEST(IntrinsicParametersTest, DefaultConstruction) {
    IntrinsicParameters params;
    // Should construct without error
    // Default values are implementation-defined
}

TEST(IntrinsicParametersTest, SetAllParameters) {
    IntrinsicParameters params;
    params.fx = 500.0f;
    params.fy = 500.0f;
    params.cx = 256.0f;
    params.cy = 256.0f;
    params.codx = 0.0f;
    params.cody = 0.0f;
    params.k1 = 0.1f;
    params.k2 = 0.01f;
    params.k3 = 0.001f;
    params.k4 = 0.0001f;
    params.k5 = 0.00001f;
    params.k6 = 0.000001f;
    params.p1 = 0.0f;
    params.p2 = 0.0f;

    EXPECT_FLOAT_EQ(params.fx, 500.0f);
    EXPECT_FLOAT_EQ(params.fy, 500.0f);
    EXPECT_FLOAT_EQ(params.cx, 256.0f);
    EXPECT_FLOAT_EQ(params.cy, 256.0f);
    EXPECT_FLOAT_EQ(params.k1, 0.1f);
    EXPECT_FLOAT_EQ(params.k2, 0.01f);
}

TEST(IntrinsicParametersTest, ReasonableValues) {
    IntrinsicParameters params;
    params.fx = 500.0f;
    params.fy = 500.0f;
    params.cx = 256.0f;
    params.cy = 256.0f;

    // Check reasonable ranges for 512x512 sensor
    EXPECT_GT(params.fx, 0.0f);
    EXPECT_GT(params.fy, 0.0f);
    EXPECT_GE(params.cx, 0.0f);
    EXPECT_GE(params.cy, 0.0f);
    EXPECT_LT(params.cx, 512.0f);
    EXPECT_LT(params.cy, 512.0f);
}

// ============================================================================
// Metadata Tests
// ============================================================================

TEST(MetadataTest, DefaultConstruction) {
    Metadata metadata{}; // Zero-initialize with {}
    EXPECT_EQ(metadata.height, 0);
    EXPECT_EQ(metadata.width, 0);
}

TEST(MetadataTest, SetBasicFields) {
    Metadata metadata;
    metadata.width = 512;
    metadata.height = 512;
    metadata.bitsInDepth = 16;
    metadata.bitsInAb = 16;
    metadata.bitsInConfidence = 8;

    EXPECT_EQ(metadata.width, 512);
    EXPECT_EQ(metadata.height, 512);
    EXPECT_EQ(metadata.bitsInDepth, 16);
    EXPECT_EQ(metadata.bitsInAb, 16);
    EXPECT_EQ(metadata.bitsInConfidence, 8);
}

TEST(MetadataTest, StreamOutput) {
    Metadata metadata{}; // Zero-initialize
    metadata.width = 512;
    metadata.height = 512;
    metadata.bitsInDepth = 16;
    metadata.bitsInAb = 16;
    metadata.bitsInConfidence = 8;
    metadata.frameNumber = 1;

    std::ostringstream oss;
    oss << metadata;

    // Should produce some output
    EXPECT_FALSE(oss.str().empty());
    EXPECT_TRUE(oss.str().find("512") != std::string::npos);
}

// ============================================================================
// Integration Tests - Combined Operations
// ============================================================================

TEST(IntegrationTest, BuildCompleteFrameDetails) {
    // Simulate building a complete frame configuration
    FrameDetails details;
    details.width = 512;
    details.height = 512;
    details.type = "QMP";
    details.cameraMode = "qmp_pcm_native";

    // Add depth
    FrameDataDetails depth;
    depth.type = "depth";
    depth.width = 512;
    depth.height = 512;
    depth.subelementSize = 2;
    depth.subelementsPerElement = 1;
    details.dataDetails.push_back(depth);

    // Add AB
    FrameDataDetails ab;
    ab.type = "ab";
    ab.width = 512;
    ab.height = 512;
    ab.subelementSize = 2;
    ab.subelementsPerElement = 1;
    details.dataDetails.push_back(ab);

    // Add confidence
    FrameDataDetails conf;
    conf.type = "conf";
    conf.width = 512;
    conf.height = 512;
    conf.subelementSize = 4; // float
    conf.subelementsPerElement = 1;
    details.dataDetails.push_back(conf);

    // Verify complete configuration
    EXPECT_EQ(details.dataDetails.size(), 3);

    // Calculate total memory needed
    uint32_t totalBytes = 0;
    for (const auto &dataDetail : details.dataDetails) {
        totalBytes += dataDetail.width * dataDetail.height *
                      dataDetail.subelementSize *
                      dataDetail.subelementsPerElement;
    }

    // Expected: (512*512*2) + (512*512*2) + (512*512*4) = 524288 + 524288 + 1048576 = 2097152
    EXPECT_EQ(totalBytes, 2097152);
}

TEST(IntegrationTest, StatusPropagation) {
    // Simulate status propagation pattern
    auto checkOperation = [](bool success) -> Status {
        return success ? Status::OK : Status::GENERIC_ERROR;
    };

    Status result1 = checkOperation(true);
    EXPECT_EQ(result1, Status::OK);

    Status result2 = checkOperation(false);
    EXPECT_EQ(result2, Status::GENERIC_ERROR);
    EXPECT_NE(result2, Status::OK);

    // Chain checks
    bool operationsSuccessful =
        (result1 == Status::OK) && (result2 != Status::OK);
    EXPECT_TRUE(operationsSuccessful);
}

TEST(IntegrationTest, ModeSwitching) {
    // Simulate mode switching between MP and QMP
    FrameDetails currentMode;
    currentMode.width = 512;
    currentMode.height = 512;
    currentMode.type = "QMP";

    FrameDetails newMode;
    newMode.width = 1024;
    newMode.height = 1024;
    newMode.type = "MP";

    // Check if mode switch is needed
    bool needsSwitch = (currentMode.width != newMode.width) ||
                       (currentMode.height != newMode.height);
    EXPECT_TRUE(needsSwitch);

    // Simulate switch
    currentMode = newMode;
    EXPECT_EQ(currentMode.width, 1024);
    EXPECT_EQ(currentMode.height, 1024);
    EXPECT_EQ(currentMode.type, "MP");
}
