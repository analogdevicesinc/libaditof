/*
 * MIT License
 *
 * Copyright (c) 2025 Analog Devices, Inc.
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
#ifndef CAMERA_DEFINITIONS_H
#define CAMERA_DEFINITIONS_H

#include "connections.h"
#include "frame_definitions.h"
#include "status_definitions.h"

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

/**
 * @brief Namespace aditof
 */
namespace aditof {

class Frame;

/**
 * @struct IntrinsicParameters
 * @brief Describes the intrinsic parameters of a camera.
 */
struct IntrinsicParameters {
    /**
     * @brief fx
     */
    float fx;

    /**
     * @brief fy
     */
    float fy;

    /**
     * @brief cx
     */
    float cx;

    /**
     * @brief cy
     */
    float cy;

    /**
     * @brief codx
     */
    float codx;

    /**
     * @brief cody
     */
    float cody;

    /**
     * @brief k1
     */
    float k1;

    /**
     * @brief k2
     */
    float k2;

    /**
     * @brief k3
     */
    float k3;

    /**
     * @brief k4
     */
    float k4;

    /**
     * @brief k5
     */
    float k5;

    /**
     * @brief k6
     */
    float k6;

    /**
     * @brief p2
     */
    float p2;

    /**
     * @brief p1
     */
    float p1;
};

/**
 * @struct CameraDetails
 * @brief Describes the properties of a camera.
 */
struct CameraDetails {
    /**
     * @brief Camera identification
     */
    std::string cameraId;

    /**
     * @brief The mode in which the camera operates
     */
    std::int8_t mode;

    /**
     * @brief Details about the frames that camera is capturing
     */
    FrameDetails frameType;

    /**
     * @brief The type of connection with the camera
     */
    ConnectionType connection;

    /**
     * @brief Details about the intrinsic parameters of the camera
     */
    IntrinsicParameters intrinsics;

    /**
     * @brief The maximum distance (in millimeters) the camera can measure in
     * the current operating mode. This is currently unused!
     */
    int maxDepth;

    /**
     * @brief The minimum distance (in millimeters) the camera can measure in
     * the current operating mode. This is currently unused!
     */
    int minDepth;

    /**
     * @brief The number of bits used for representing one pixel data. This is currently unused!
     */
    int bitCount;

    /**
     * @brief The U-Boot version that is installed on the embedded system that the camera is attached to.
     */
    std::string uBootVersion;

    /**
     * @brief The kernel version that is installed on the embedded system that the camera is attached to.
     */
    std::string kernelVersion;

    /**
     * @brief The SD card image version on the embedded system that the camera is attached to.
     */
    std::string sdCardImageVersion;

    /**
     * @brief The serial number of camera
     */
    std::string serialNumber;
};

/**
 * @enum ImagerType
 * @brief Types of imagers
 */
enum class ImagerType {
    UNSET,    //!< Value for when the type is unset
    ADSD3100, //!< The ADSD3100 imager
    ADSD3030, //!< The ADSD3030 imager
    ADTF3080, //!< The ADTF3080 imager
    ADTF3066, //!< The ADTF3066 imager
};

/**
 * @brief Types of imagers based on ControlValue
 */

const std::map<ImagerType, std::string> ControlValue{
    {ImagerType::ADSD3100, "1"},
    {ImagerType::ADSD3030, "2"},
    {ImagerType::ADTF3080, "3"},
    {ImagerType::ADTF3066, "4"}};
/**
 * @brief Types of imagers 
 */
const std::map<ImagerType, std::string> imagerType{
    {ImagerType::ADSD3100, "adsd3100"},
    {ImagerType::ADSD3030, "adsd3030"},
    {ImagerType::ADTF3080, "adtf3080"},
    {ImagerType::ADTF3066, "adtf3066"}};

} // namespace aditof

#endif // CAMERA_DEFINITIONS_H
