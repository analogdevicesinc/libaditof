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
#ifndef FRAME_DEFINITIONS_H
#define FRAME_DEFINITIONS_H

#include <iostream>
#include <string>
#include <vector>

/**
 * @brief Namespace aditof
 */
namespace aditof {

/**
 * @struct FrameDataDetails
 * @brief Describes the properties of a data that embedded within the frame
 */
struct FrameDataDetails {
    /**
     * @brief The type of data that can be found in a frame. For example it
     * could be depth data or IR data, etc.
     */
    std::string type;

    /**
     * @brief The width of the frame data
     */
    unsigned int width;

    /**
     * @brief The height of the frame data
     */
    unsigned int height;

    /**
     * @brief The size in bytes of a sub-element.
     * A sub-element is a sub-component of an element. All sub-elements make up
     * an element. For instance a 3D point (which is an element) has X, Y and Z
     * coordinates which are sub-elements of the 3D point.
     */
    unsigned int subelementSize;

    /**
     * @brief The number of sub-elements that an element has.
     * An element is the smallest part of the image (a.k.a. captured scene) that
     * together with other elements make up the entire image.
     */
    unsigned int subelementsPerElement;

    /**
     * @brief The total number of bytes that the data has.
     * This can be useful when copying data to another location or when saving
     * data to a file or any other usecase where the size in bytes needs to be
     * known.
     */
    unsigned int bytesCount;
};

/**
 * @struct FrameDetails
 * @brief Describes the properties of a frame.
 */
struct FrameDetails {
    /**
     * @brief The type of the frame. Can be one of the types provided by the
     * camera.
     */
    std::string type;

    /**
     * @brief A frame can have multiple types of data. For example it could
     * hold data about depth and/or data about IR.
     */
    std::vector<FrameDataDetails> dataDetails;

    /**
     * @brief The mode the camera was set when the frame was captured.
     */
    std::string cameraMode;

    /**
     * @brief The width of the frame.
     */
    unsigned int width;

    /**
     * @brief The height of the frame.
     */
    unsigned int height;

    /**
     * @brief totalCaptures or subframes in a frame
     */
    uint8_t totalCaptures;

    /**
    * @brief is a passive IR frame appended
    */
    bool passiveIRCaptured;
};

/**
 * @struct Point3I_sdk
 * @brief Holds the xyz values of a frame
 */
struct Point3I_sdk {
    int16_t a; //!< X Information
    int16_t b; //!< Y Information
    int16_t c; //!< Z Information
};

#pragma pack(push, 1)
/**
 * @struct Metadata
 * @brief Contains all of the metadata components
 */
struct Metadata {

    /**
    * @brief Width of frame
    */
    uint16_t width;

    /**
    * @brief Height of frame
    */
    uint16_t height;

    /**
    * @brief ADSD3500 Output Configuration:
    * 0 Full Depth Frame
    * 1 Phase Frame (Partial Depth)
    * 2 AB Frame
    * 3 Confidence Frame
    * 4 Depth AB Interleaved
    * 5 Phase and AB Interleaved
    * 6 Phase, JBLF Confidence and AB Interleaved
    * 7 Depth, Confidence and AB Interleaved
    */
    uint8_t outputConfiguration;

    /**
    * @brief Number of bits in depth
    */
    uint8_t bitsInDepth;

    /**
    * @brief Number of bits in AB
    */
    uint8_t bitsInAb;

    /**
    * @brief Number of bits in confidence
    */
    uint8_t bitsInConfidence;

    /**
    * @brief invalidPhaseValue:
    * In partial depth case, the host must know the invalid phase value used by the ADSD3500, which is used for invalidation during full depth compute.
    */
    uint16_t invalidPhaseValue;

    /**
    * @brief frequencyIndex: Stores index of the frequency for which the phase frame is outputted.
    */
    uint8_t frequencyIndex;

    /**
    * @brief abFrequencyIndex:
    * AB Frequency Index:
    * 0 AB of frequency 0
    * 1 AB of frequency 1
    * 2 AB of frequency 2
    * 3 AB Averaged
    */
    uint8_t abFrequencyIndex;

    /**
    * @brief Frame number
    */
    uint32_t frameNumber;

    /**
    * @brief Imager mode
    */
    uint8_t imagerMode;

    /**
    * @brief number of phases:
    * Number of phases in the input raw capture fed to the ADSD3500
    */
    uint8_t numberOfPhases;

    /**
    * @brief number of frequencies:
    * Number of frequencies in the input raw capture fed to the ADSD3500.
    */
    uint8_t numberOfFrequencies;

    /**
    * @brief True if xyz is being generated for the current frame. (set by sdk)
    */
    uint8_t xyzEnabled;

    /**
    * @brief elapsedTimeFractionalValue:
    * 32-bit fractional value out of total elapsed time.
    */
    uint32_t elapsedTimeFractionalValue;

    /**
    * @brief elapsedTimeSecondsValue:
    * 32-bit seconds value out of total elapsed time.
    */
    uint32_t elapsedTimeSecondsValue;

    /**
    * @brief Sensor temperature in degrees Celsius
    */
    int32_t sensorTemperature;

    /**
    * @brief Laser temperature in degrees Celsius
    */
    int32_t laserTemperature;
};
#pragma pack(pop)

/**
 * @brief prints human readable metadata structure
 */
inline std::ostream &operator<<(std::ostream &o, const struct Metadata &meta) {
    o << "\tWidth: " << meta.width << "\tHeight: " << meta.height
      << "\tOutputConfiguration: "
      << static_cast<unsigned int>(meta.outputConfiguration)
      << "\tBitsInDepth: " << static_cast<unsigned int>(meta.bitsInDepth)
      << "\tBitsInAb: " << static_cast<unsigned int>(meta.bitsInAb)
      << "\tBitsInConfidenc: "
      << static_cast<unsigned int>(meta.bitsInConfidence)
      << "\tInvalidPhaseValue: " << meta.invalidPhaseValue
      << "\tFrequencyIndex: " << static_cast<unsigned int>(meta.frequencyIndex)
      << "\tFrameNumber: " << meta.frameNumber
      << "\tImagerMode: " << static_cast<unsigned int>(meta.imagerMode)
      << "\tNumberOfPhases: " << static_cast<unsigned int>(meta.numberOfPhases)
      << "\tNumberOfFrequencies: "
      << static_cast<unsigned int>(meta.numberOfFrequencies)
      << "\tXYZEnabled: " << static_cast<unsigned int>(meta.xyzEnabled)
      << "\tElapsedTimeFractionalValue: " << meta.elapsedTimeFractionalValue
      << "\tElapsedTimeSecondsValue: " << meta.elapsedTimeSecondsValue
      << "\tSensorTemperature: " << meta.sensorTemperature
      << "\tLaserTemperature: " << meta.laserTemperature << "\n";
    return o;
}

} // namespace aditof

#endif // FRAME_DEFINITIONS_H
