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
#include "aditof/frame_operations.h"

#include <aditof/log.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <memory>
#include <unordered_map>

static const int skMetaDataBytesCount = 128;

struct FrameImpl::ImplData {
    std::unordered_map<std::string, uint16_t *> m_dataLocations;
    std::shared_ptr<uint16_t[]> m_allData;
    size_t allDataNbBytes;
};

/**
 * @brief Default constructor for FrameImpl.
 *
 * Initializes a new Frame object with default implementation data.
 * The internal data structure is allocated on the heap using a unique_ptr.
 * The details flag is initially set to false, indicating frame details have not
 * yet been configured.
 */
FrameImpl::FrameImpl()
    : m_implData(std::make_unique<FrameImpl::ImplData>()),
      m_detailsConfigured(false){};
/**
 * @brief Destructor for FrameImpl.
 *
 * Cleans up the internal implementation data. The unique_ptr m_implData
 * automatically handles deallocation of the ImplData structure and associated
 * frame data buffers.
 */
FrameImpl::~FrameImpl() = default;

/**
 * @brief Copy constructor for FrameImpl.
 *
 * Creates a new FrameImpl instance by copying another FrameImpl object.
 * Allocates new frame data and performs a deep copy of the frame's buffer
 * contents, details, and configuration state. This ensures independence
 * between the original and copied Frame objects.
 *
 * @param op The FrameImpl object to copy from.
 */
FrameImpl::FrameImpl(const FrameImpl &op) {
    allocFrameData(op.m_details);
    memcpy(m_implData->m_allData.get(), op.m_implData->m_allData.get(),
           m_implData->allDataNbBytes);
    m_details = op.m_details;
    m_detailsConfigured = op.m_detailsConfigured;
}

/**
 * @brief Assignment operator for FrameImpl.
 *
 * Assigns a FrameImpl object to another FrameImpl object. Performs a self-check
 * to prevent unnecessary copying. Allocates new frame data and performs a
 * deep copy of the frame's buffer contents, details, and configuration state.
 * This ensures independence between the assigned and source Frame objects.
 *
 * @param op The FrameImpl object to assign from.
 * @return A reference to the assigned FrameImpl object (*this).
 */
FrameImpl &FrameImpl::operator=(const FrameImpl &op) {
    if (this != &op) {
        allocFrameData(op.m_details);
        memcpy(m_implData->m_allData.get(), op.m_implData->m_allData.get(),
               m_implData->allDataNbBytes);
        m_details = op.m_details;
        m_detailsConfigured = op.m_detailsConfigured;
    }

    return *this;
}

/**
 * @brief Sets the frame details and allocates memory for frame data.
 *
 * Configures the frame with the specified FrameDetails, which defines the
 * data layout (depth, AB, confidence, metadata, etc.). Automatically allocates
 * the necessary memory to store the frame data based on the details specification.
 * Also stores bit depth information for confidence and AB channels for later
 * processing. Logs a warning if the Frame is being reused with different details
 * (e.g., during mode changes) and recommends creating a fresh Frame object.
 *
 * @param details The FrameDetails object containing frame configuration
 *                 (dimensions, data types, etc.).
 * @param m_bitsInConf The bit depth for confidence data (default 8 bits).
 * @param m_bitsInAB The bit depth for AB (amplitude/brightness) data
 *                   (default 8 bits).
 * @return Status::OK on success, or an error status if configuration fails.
 *
 * @note Calling this multiple times with different dimensions will log a warning
 *       but will still succeed. Consider creating a new Frame object instead
 *       for mode changes.
 */
aditof::Status FrameImpl::setDetails(const aditof::FrameDetails &details,
                                     const uint8_t &m_bitsInConf,
                                     const uint8_t &m_bitsInAB) {
    using namespace aditof;
    Status status = Status::OK;

    if (details == m_details) {
        LOG(INFO) << "Same details provided. Doing nothing.";
        return status;
    }

    // Warn if Frame is being reused with different details (mode change)
    if (m_detailsConfigured && m_details.width > 0) {
        LOG(WARNING) << "Frame object is being reused across mode changes! "
                     << "Old: " << m_details.width << "x" << m_details.height
                     << ", New: " << details.width << "x" << details.height
                     << ". Consider using 'frame = aditof::Frame();' to create "
                        "a fresh Frame object.";
    }

    allocFrameData(details);
    m_details = details;
    m_detailsConfigured = true;

    return status;
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
aditof::Status FrameImpl::getDetails(aditof::FrameDetails &details) const {
    details = m_details;

    return aditof::Status::OK;
}

/**
 * @brief Retrieves data details for a specific data type within the frame.
 *
 * Searches for and returns the FrameDataDetails for the specified data type
 * (e.g., "depth", "ab", "confidence", "metadata"). This provides information
 * about the layout, dimensions, and format of that specific data within the frame.
 *
 * @param dataType The name of the data type to query (e.g., "depth", "ab",
 *                 "confidence", "metadata").
 * @param details Output parameter where the FrameDataDetails for the requested
 *                type will be copied.
 * @return Status::OK if the data type is found, Status::INVALID_ARGUMENT if not.
 */
aditof::Status
FrameImpl::getDataDetails(const std::string &dataType,
                          aditof::FrameDataDetails &details) const {
    auto detailsIter =
        std::find_if(m_details.dataDetails.begin(), m_details.dataDetails.end(),
                     [&dataType](const aditof::FrameDataDetails &details) {
                         return dataType == details.type;
                     });
    if (detailsIter == m_details.dataDetails.end()) {
        LOG(WARNING) << "Could not find any details for type: " << dataType;
        return aditof::Status::INVALID_ARGUMENT;
    }

    details = *detailsIter;

    return aditof::Status::OK;
}

/**
 * @brief Retrieves a pointer to the frame data for a specific data type.
 *
 * Returns a pointer to the internal frame data buffer for the specified data type.
 * The pointer points to the beginning of that data type's region within the
 * allocated frame buffer. Used to access raw depth, AB, confidence, or metadata.
 *
 * @param dataType The name of the data type to retrieve (e.g., "depth", "ab",
 *                 "confidence", "xyz", "metadata").
 * @param dataPtr Output parameter where the pointer to the data will be stored.
 * @return Status::OK if the data type exists, Status::INVALID_ARGUMENT if not
 *         supported or available.
 */
aditof::Status FrameImpl::getData(const std::string &dataType,
                                  uint16_t **dataPtr) {
    using namespace aditof;
    if (m_implData->m_dataLocations.count(dataType) > 0) {
        *dataPtr = m_implData->m_dataLocations[dataType];
    } else {
        dataPtr = nullptr;
        if (dataType != "metadata")
            LOG(ERROR) << dataType << " is not supported by this frame!";
        return Status::INVALID_ARGUMENT;
    }

    return Status::OK;
}

/**
 * @brief Checks if the frame contains data for a specific data type.
 *
 * Determines whether the frame has allocated and initialized data for the
 * specified data type.
 *
 * @param dataType The name of the data type to check (e.g., "depth", "ab",
 *                 "confidence", "metadata").
 * @return true if the data type is present and non-null, false otherwise.
 */
bool FrameImpl::haveDataType(const std::string &dataType) {
    if (m_implData->m_dataLocations.count(dataType) > 0) {
        return (m_implData->m_dataLocations[dataType] != nullptr);
    }

    return false;
}

/**
 * @brief Retrieves frame data details by name from a FrameDetails structure.
 *
 * Searches within the given FrameDetails object for the FrameDataDetails
 * matching the specified name/type. Useful for extracting specific data
 * layout information (e.g., depth, confidence, metadata) from a frame
 * configuration.
 *
 * @param details The FrameDetails structure to search within.
 * @param name The name of the data type to locate (e.g., "depth", "ab",
 *             "confidence", "metadata").
 * @return The FrameDataDetails corresponding to the name. If not found,
 *         returns an uninitialized FrameDataDetails object and logs a warning.
 *
 * @note This is a static utility method that does not require frame state.
 */
aditof::FrameDataDetails
FrameImpl::getFrameDetailByName(const aditof::FrameDetails &details,
                                const std::string name) {
    auto frame_detail =
        std::find_if(details.dataDetails.begin(), details.dataDetails.end(),
                     [&name](const aditof::FrameDataDetails frame_detail) {
                         return frame_detail.type == name;
                     });

    if (frame_detail == details.dataDetails.end()) {
        LOG(WARNING) << "Could not find any attribute with name: " << name;
    }

    return *frame_detail;
}

/**
 * @brief Allocates memory for frame data based on the specified FrameDetails.
 *
 * Calculates the total memory required to store all frame data types
 * (depth, AB, confidence, metadata, etc.) as defined in the FrameDetails,
 * and allocates a contiguous buffer. Sets up a mapping (m_dataLocations)
 * that stores pointers to each data type's region within the allocated buffer.
 *
 * @param details The FrameDetails object that specifies the frame layout,
 *                dimensions, and data types to allocate.
 *
 * @note The allocation uses a shared_ptr with a custom deleter for safe
 *       memory management. Supports data types: "metadata", "xyz", "conf",
 *       "ab", "depth", and "frameData" (full buffer).
 */
void FrameImpl::allocFrameData(const aditof::FrameDetails &details) {
    using namespace aditof;
    unsigned long int totalSize = 0;
    unsigned long int pos = 0;
    uint16_t embed_hdr_length = skMetaDataBytesCount;

    auto getSubframeSize = [embed_hdr_length](FrameDataDetails frameDetail) {
        unsigned long int sz;

        if (frameDetail.type == "metadata") {
            sz = (unsigned long int)(embed_hdr_length / sizeof(uint16_t));
        } else if (frameDetail.type == "xyz") {
            sz =
                (unsigned long int)(frameDetail.height * frameDetail.width * 3);
        } else if (frameDetail.type == "conf") {
            sz = (unsigned long int)(frameDetail.height * frameDetail.width *
                                     sizeof(float) / sizeof(uint16_t));
        } else if (frameDetail.type == "ab") {
            sz = (unsigned long int)(frameDetail.height * frameDetail.width);
        } else {
            sz = (unsigned long int)(frameDetail.height * frameDetail.width);
        }
        return sz;
    };

    for (FrameDataDetails frameDetail : details.dataDetails) {
        totalSize += getSubframeSize(frameDetail);
    }

    //store pointers to the contents described by FrameDetails
    m_implData->m_allData = std::shared_ptr<uint16_t[]>(
        new uint16_t[totalSize], // Allocate the array
        [](uint16_t *p) {
            delete[] p;
        } // Custom deleter to ensure correct deallocation
    );

    m_implData->m_dataLocations.emplace(
        "frameData", m_implData->m_allData.get()); //frame data
    for (FrameDataDetails frameDetail : details.dataDetails) {
        m_implData->m_dataLocations.emplace(
            frameDetail.type, m_implData->m_allData.get() + pos); //raw data

        pos += getSubframeSize(frameDetail);
    }
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
aditof::Status FrameImpl::getMetadataStruct(aditof::Metadata &metadata) const {
    using namespace aditof;

    uint8_t *header;
    if (m_implData->m_dataLocations.count("metadata") > 0) {
        header = reinterpret_cast<uint8_t *>(
            m_implData->m_dataLocations["metadata"]);
    } else {
        return aditof::Status::UNAVAILABLE;
    }

    memcpy(&metadata, header, sizeof(Metadata));

    return aditof::Status::OK;
}