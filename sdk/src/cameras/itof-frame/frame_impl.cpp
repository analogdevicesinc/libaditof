/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Analog Devices, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "frame_impl.h"
#include "aditof/frame_operations.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#endif
#include <memory>
#include <unordered_map>

static const int skMetaDataBytesCount = 128;

struct FrameImpl::ImplData {
    std::unordered_map<std::string, uint16_t *> m_dataLocations;
    std::shared_ptr<uint16_t[]> m_allData;
    size_t allDataNbBytes;
};

FrameImpl::FrameImpl() : m_implData(std::make_unique<FrameImpl::ImplData>()){};
FrameImpl::~FrameImpl() = default;

FrameImpl::FrameImpl(const FrameImpl &op) {
    allocFrameData(op.m_details);
    memcpy(m_implData->m_allData.get(), op.m_implData->m_allData.get(),
           m_implData->allDataNbBytes);
    m_details = op.m_details;
}

FrameImpl &FrameImpl::operator=(const FrameImpl &op) {
    if (this != &op) {
        allocFrameData(op.m_details);
        memcpy(m_implData->m_allData.get(), op.m_implData->m_allData.get(),
               m_implData->allDataNbBytes);
        m_details = op.m_details;
    }

    return *this;
}

aditof::Status FrameImpl::setDetails(const aditof::FrameDetails &details) {
    using namespace aditof;
    Status status = Status::OK;

    if (details == m_details) {
        LOG(INFO) << "Same details provided. Doing nothing.";
        return status;
    }

    allocFrameData(details);
    m_details = details;

    return status;
}

aditof::Status FrameImpl::getDetails(aditof::FrameDetails &details) const {
    details = m_details;

    return aditof::Status::OK;
}

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

aditof::Status FrameImpl::getData(const std::string &dataType,
                                  uint16_t **dataPtr) {
    using namespace aditof;
    if (m_implData->m_dataLocations.count(dataType) > 0) {
        *dataPtr = m_implData->m_dataLocations[dataType];
    } else {
        dataPtr = nullptr;
        if (dataType !=
            "metadata") // TO DO: Silence this for now, handle it later
            LOG(ERROR) << dataType << " is not supported by this frame!";
        return Status::INVALID_ARGUMENT;
    }

    return Status::OK;
}

bool FrameImpl::haveDataType(const std::string &dataType) {
    if (m_implData->m_dataLocations.count(dataType) > 0) {
        return (m_implData->m_dataLocations[dataType] != nullptr);
    }

    return false;
}

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

void FrameImpl::allocFrameData(const aditof::FrameDetails &details) {
    using namespace aditof;
    unsigned long int totalSize = 0;
    unsigned long int pos = 0;
    uint16_t embed_hdr_length = skMetaDataBytesCount;
    uint8_t total_captures = 0;

    auto getSubframeSize = [embed_hdr_length,
                            total_captures](FrameDataDetails frameDetail) {
        if (frameDetail.type == "metadata") {
            unsigned long int sz =
                (unsigned long int)(embed_hdr_length / sizeof(uint16_t));
            //LOG(INFO) << "metadata: " << sz * sizeof(uint16_t) << " bytes";
            return sz;
        } else if (frameDetail.type == "xyz") {
            unsigned long int sz =
                (unsigned long int)(frameDetail.height * frameDetail.width * 3);
            //LOG(INFO) << "XYZ: " << sz * sizeof(uint16_t) << " bytes";
            return sz;
        } else if (frameDetail.type == "conf") {
            unsigned long int sz =
                (unsigned long int)(frameDetail.height * frameDetail.width *
                                    sizeof(float) / sizeof(uint16_t));
            //LOG(INFO) << "Conf: " << sz * sizeof(uint16_t) << " bytes";
            return sz;
        } else {
            unsigned long int sz =
                (unsigned long int)(frameDetail.height * frameDetail.width);
            //LOG(INFO) << "Other: " << sz * sizeof(uint16_t) << " bytes";
            return sz;
        }
    };

    //compute total size TODO this could be precomputed TBD @dNechita
    for (FrameDataDetails frameDetail : details.dataDetails) {
        totalSize += getSubframeSize(frameDetail);
    }

    //LOG(INFO) << "Allocating frame data of total size: " << totalSize * sizeof(uint16_t) << " bytes";

    //store pointers to the contents described by FrameDetails
    m_implData->m_allData = std::shared_ptr<uint16_t[]>(
        new uint16_t[totalSize], // Allocate the array
        [](uint16_t *p) {
            delete[] p;
        } // Custom deleter to ensure correct deallocation
    );

    //TODO wouldn`t it be safer to store relative position to .get() instead of absolute address ? TBD @dNechita
    m_implData->m_dataLocations.emplace(
        "frameData", m_implData->m_allData.get()); //frame data
    for (FrameDataDetails frameDetail : details.dataDetails) {
        m_implData->m_dataLocations.emplace(
            frameDetail.type, m_implData->m_allData.get() + pos); //raw data

        pos += getSubframeSize(frameDetail);
    }
}

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