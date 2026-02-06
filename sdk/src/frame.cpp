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
#include "frame_impl.h"
#include <aditof/frame.h>
#include <memory>

namespace aditof {

/**
 * @brief Default constructor for Frame.
 *
 * Creates a new Frame object and initializes the internal FrameImpl
 * implementation. This class follows the PIMPL (Pointer to IMPLementation)
 * idiom to hide implementation details and maintain ABI stability.
 */
Frame::Frame() : m_impl(std::make_unique<FrameImpl>()) {}

/**
 * @brief Destructor for Frame.
 *
 * Cleans up the Frame object and releases the internal implementation.
 * All allocated frame data is automatically freed.
 */
Frame::~Frame() = default;

/**
 * @brief Move constructor for Frame.
 *
 * Transfers ownership of the internal implementation from another Frame object.
 * The source Frame is left in a valid but unspecified state after the move.
 */
Frame::Frame(Frame &&) noexcept = default;

/**
 * @brief Move assignment operator for Frame.
 *
 * Transfers ownership of the internal implementation from another Frame object.
 * The source Frame is left in a valid but unspecified state after the move.
 *
 * @return A reference to this Frame object.
 */
Frame &Frame::operator=(Frame &&) noexcept = default;

/**
 * @brief Copy constructor for Frame.
 *
 * Creates a new Frame by performing a deep copy of another Frame object.
 * All frame data, details, and configuration are copied to ensure independence
 * between the original and copied Frame objects.
 *
 * @param op The Frame object to copy from.
 */
Frame::Frame(const Frame &op)
    : m_impl(std::make_unique<FrameImpl>(*op.m_impl)) {}

/**
 * @brief Copy assignment operator for Frame.
 *
 * Assigns a Frame object by performing a deep copy of another Frame object.
 * Performs a self-assignment check to prevent unnecessary copying. All frame
 * data, details, and configuration are copied to ensure independence.
 *
 * @param op The Frame object to copy from.
 * @return A reference to this Frame object.
 */
Frame &Frame::operator=(const Frame &op) {
    if (this != &op) {
        m_impl = std::make_unique<FrameImpl>(*op.m_impl);
    }
    return *this;
}

/**
 * @brief Sets the frame details and allocates memory for frame data.
 *
 * Configures the frame with the specified FrameDetails, which defines the
 * data layout (depth, AB, confidence, metadata, etc.). Automatically allocates
 * the necessary memory to store the frame data based on the details specification.
 * Also stores bit depth information for confidence and AB channels.
 *
 * @param details The FrameDetails object containing frame configuration
 *                (dimensions, data types, etc.).
 * @param m_bitsInConf The bit depth for confidence data (default varies by mode).
 * @param m_bitsInAB The bit depth for AB (amplitude/brightness) data
 *                   (default varies by mode).
 * @return Status::OK on success, or an error status if configuration fails.
 */
Status Frame::setDetails(const FrameDetails &details,
                         const uint8_t &m_bitsInConf,
                         const uint8_t &m_bitsInAB) {
    return m_impl->setDetails(details, m_bitsInConf, m_bitsInAB);
}

/**
 * @brief Retrieves the current frame details.
 *
 * Returns the FrameDetails object associated with this Frame, which defines
 * the frame's data layout, dimensions, and supported data types.
 *
 * @param details Output parameter where the current FrameDetails will be copied.
 * @return Status::OK on success.
 */
Status Frame::getDetails(FrameDetails &details) const {
    return m_impl->getDetails(details);
}

/**
 * @brief Retrieves data details for a specific data type within the frame.
 *
 * Searches for and returns the FrameDataDetails for the specified data type
 * (e.g., "depth", "ab", "confidence", "metadata"). This provides information
 * about the layout, dimensions, and format of that specific data within the frame.
 *
 * @param dataType The name of the data type to query (e.g., "depth", "ab",
 *                 "confidence", "metadata", "xyz").
 * @param details Output parameter where the FrameDataDetails for the requested
 *                type will be copied.
 * @return Status::OK if the data type is found, Status::INVALID_ARGUMENT if not.
 */
Status Frame::getDataDetails(const std::string &dataType,
                             FrameDataDetails &details) const {
    return m_impl->getDataDetails(dataType, details);
}

/**
 * @brief Retrieves a pointer to the frame data for a specific data type.
 *
 * Returns a pointer to the internal frame data buffer for the specified data type.
 * The pointer points to the beginning of that data type's region within the
 * allocated frame buffer. Used to access raw depth, AB, confidence, XYZ, or metadata.
 *
 * @param dataType The name of the data type to retrieve (e.g., "depth", "ab",
 *                 "confidence", "xyz", "metadata").
 * @param dataPtr Output parameter where the pointer to the data will be stored.
 * @return Status::OK if the data type exists, Status::INVALID_ARGUMENT if not
 *         supported or available.
 */
Status Frame::getData(const std::string &dataType, uint16_t **dataPtr) {
    return m_impl->getData(dataType, dataPtr);
}

/**
 * @brief Checks if the frame contains data for a specific data type.
 *
 * Determines whether the frame has allocated and initialized data for the
 * specified data type. Useful for checking availability before calling getData().
 *
 * @param dataType The name of the data type to check (e.g., "depth", "ab",
 *                 "confidence", "metadata", "xyz").
 * @return true if the data type is present and non-null, false otherwise.
 */
bool Frame::haveDataType(const std::string &dataType) {
    return m_impl->haveDataType(dataType);
}

/**
 * @brief Retrieves the frame's embedded metadata structure.
 *
 * Extracts and returns the Metadata structure from the frame's metadata buffer.
 * The metadata includes embedded header information such as frame mode,
 * timestamp, sensor configuration, and other ISP-generated metadata.
 *
 * @param metadata Output parameter where the Metadata structure will be copied
 *                 from the frame's metadata buffer.
 * @return Status::OK on success, Status::UNAVAILABLE if the frame does not
 *         have a metadata data type allocated.
 */
Status Frame::getMetadataStruct(Metadata &metadata) const {
    return m_impl->getMetadataStruct(metadata);
}

} // namespace aditof
