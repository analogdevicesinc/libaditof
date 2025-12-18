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
#ifndef SENSOR_DEFINITIONS_H
#define SENSOR_DEFINITIONS_H

#include "aditof/connections.h"

#include <iostream>
#include <string>
#include <vector>

/**
 * @brief Namespace aditof
 */
namespace aditof {

/**
 * @struct SensorDetails
 * @brief Provides details about the device
 */
struct SensorDetails {
    /**
     * @brief The sensor identification data to be used to differentiate between sensors.
     * When on target, id is set to video driver path. When on network, id is set to the IP of the target.
     */
    std::string id;

    /**
     * @brief The type of connection with the sensor
     */
    ConnectionType connectionType;
};

/**
 * @struct BitsConfiguration
 * @brief describes the bits configuration for depth, AB and conf
 */
struct BitsConfiguration {
    int depth_bits; //!< Number of bits per pixel for depth data
    int conf_bits;  //!< Number of bits per pixel for confidence data
    int ab_bits; //!< Number of bits per pixel for active brightness (AB) data
};

/**
 * @struct DriverConfiguration
 * @brief Describes the configuration of the used driver
 */
struct DriverConfiguration {
    /**
     * @brief Base width value of the frame
    */
    std::string baseWidth;
    /**
     * @brief Base heigth value of the frame
    */
    std::string baseHeigth;
    /**
     * @brief Number of phases
    */
    std::string noOfPhases;
    /**
     * @brief Stores depth data
    */
    std::string depthBits;

    /**
     * @brief Stores ab data
    */
    std::string abBits;

    /**
     * @brief Stores conf data
    */
    std::string confBits;

    /**
     * @brief Stores value from driver
    */
    std::string pixelFormat;

    /**
     * @brief Stores driver width
    */
    int driverWidth;

    /**
     * @brief Stores driver height
    */
    int driverHeigth;

    /**
     * @brief Index of two possbile values sensor values (8bit, 12/16bit)
    */
    int pixelFormatIndex;
};

/**
 * @struct DepthSensorModeDetails
 * @brief Describes the type of entire frame that a depth sensor can capture and transmit
 */
struct DepthSensorModeDetails {

    /**
     * @brief Number associated with the mode
    */
    uint8_t modeNumber;

    /**
     * @brief Stores the content of each frame
    */
    std::vector<std::string> frameContent;

    /**
     * @brief Number of phases
    */
    uint8_t numberOfPhases;

    /**
     * @brief Index of two possbile values sensor values (8bit, 12/16bit) 
    */
    int pixelFormatIndex;

    /**
     * @brief Driver width, can be used for both chipRaw and imagerRaw.
    */
    int frameWidthInBytes;

    /**
     * @brief Driver height, can be used for both chipRaw and imagerRaw.
    */
    int frameHeightInBytes;

    /**
     * @brief Processed data witdh.
    */
    int baseResolutionWidth;

    /**
     * @brief Processed data height.
    */
    int baseResolutionHeight;

    /**
     * @brief Stores the size of metadata
    */
    int metadataSize;

    /**
     * @brief set to true if the mode is PCM
    */
    int isPCM;

    /**
     * @brief Stores the content of a frame
    */
    DriverConfiguration driverConfiguration;
};

/**
 * @brief prints human readable frame details
 */
inline std::ostream &operator<<(std::ostream &o,
                                const DepthSensorModeDetails &a) {
    o << "DepthSensorModeDetails: \tN: " << a.modeNumber
      << "\tW: " << a.baseResolutionWidth << "\tH: " << a.baseResolutionHeight
      << " contains:\n";
    for (auto &content : a.frameContent) {
        o << "\t" << content;
    }
    return o;
}
} // namespace aditof

#endif // SENSOR_DEFINITIONS_H
