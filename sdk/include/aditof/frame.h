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
#ifndef FRAME_H
#define FRAME_H

#include "frame_definitions.h"
#include "sdk_exports.h"
#include "status_definitions.h"

#include <memory>
#include <vector>

class FrameImpl;

namespace aditof {

/**
 * @class Frame
 * @brief Frame of a camera.
 */
class Frame {
  public:
    /**
     * @brief Constructor
     */
    SDK_API Frame();

    /**
     * @brief Destructor
     */
    SDK_API ~Frame();

    /**
     * @brief Copy constructor
     */
    SDK_API Frame(const Frame &op);

    /**
     * @brief Copy assignment
     */
    SDK_API Frame &operator=(const Frame &op);

    /**
     * @brief Move constructor
     */
    SDK_API Frame(Frame &&) noexcept;
    /**
     * @brief Move assignment
     */
    SDK_API Frame &operator=(Frame &&) noexcept;

  public:
    /**
     * @brief Configures the frame with the given details
     * @param[in] details Frame details structure
     * @param[in] m_bitsInConf Bits per pixel in confidence data
     * @param[in] m_bitsInAB Bits per pixel in AB data
     * @return Status
     */
    SDK_API Status setDetails(const FrameDetails &details,
                              const uint8_t &m_bitsInConf,
                              const uint8_t &m_bitsInAB);

    /**
     * @brief Gets the current details of the frame
     * @param[out] details
     * @return Status
     */
    SDK_API Status getDetails(FrameDetails &details) const;

    /**
         * @brief Gets details of a type of data within the frame
         * @param dataType
         * @param[out] details
         * @return Status
         */
    SDK_API Status getDataDetails(const std::string &dataType,
                                  FrameDataDetails &details) const;

    /**
     * @brief Gets the address where the specified data is being stored
     * @param dataType
     * @param[out] dataPtr
     * @return Status
     */
    SDK_API Status getData(const std::string &dataType, uint16_t **dataPtr);

    /**
     * @brief Extracts the metadata content and returns a struct with values
     * @param[out] metadata - struct containing all metadata fields 
     * @return Status
     */
    SDK_API virtual Status getMetadataStruct(Metadata &metadata) const;

    /**
     * @brief Check if frame contains a specific data type
     * @param[in] dataType - The type of data to check for (e.g., "depth", "ab", "xyz", "conf")
     * @return True if the frame contains the specified data type, false otherwise
     */
    SDK_API virtual bool haveDataType(const std::string &dataType);

  private:
    std::unique_ptr<FrameImpl> m_impl;
};

} // namespace aditof

#endif // FRAME_H
