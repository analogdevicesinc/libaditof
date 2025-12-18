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

Frame::Frame() : m_impl(std::make_unique<FrameImpl>()) {}

Frame::~Frame() = default;

Frame::Frame(Frame &&) noexcept = default;

Frame &Frame::operator=(Frame &&) noexcept = default;

Frame::Frame(const Frame &op)
    : m_impl(std::make_unique<FrameImpl>(*op.m_impl)) {}

Frame &Frame::operator=(const Frame &op) {
    if (this != &op) {
        m_impl = std::make_unique<FrameImpl>(*op.m_impl);
    }
    return *this;
}

Status Frame::setDetails(const FrameDetails &details,
                         const uint8_t &m_bitsInConf,
                         const uint8_t &m_bitsInAB) {
    return m_impl->setDetails(details, m_bitsInConf, m_bitsInAB);
}

Status Frame::getDetails(FrameDetails &details) const {
    return m_impl->getDetails(details);
}

Status Frame::getDataDetails(const std::string &dataType,
                             FrameDataDetails &details) const {
    return m_impl->getDataDetails(dataType, details);
}

Status Frame::getData(const std::string &dataType, uint16_t **dataPtr) {
    return m_impl->getData(dataType, dataPtr);
}

bool Frame::haveDataType(const std::string &dataType) {
    return m_impl->haveDataType(dataType);
}

Status Frame::getMetadataStruct(Metadata &metadata) const {
    return m_impl->getMetadataStruct(metadata);
}

} // namespace aditof
