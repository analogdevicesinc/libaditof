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
#include "adsd3500_command_impl.h"

namespace aditof {

Adsd3500CommandImpl::Adsd3500CommandImpl(
    Adsd3500ProtocolManager *protocolManager)
    : m_protocolManager(protocolManager), m_chipStatus(0), m_imagerStatus(0) {}

Status Adsd3500CommandImpl::readCommand(uint16_t cmd, uint16_t *data,
                                        unsigned int usDelay) {
    if (!m_protocolManager) {
        return Status::GENERIC_ERROR;
    }
    return m_protocolManager->adsd3500_read_cmd(cmd, data, usDelay);
}

Status Adsd3500CommandImpl::writeCommand(uint16_t cmd, uint16_t data,
                                         unsigned int usDelay) {
    if (!m_protocolManager) {
        return Status::GENERIC_ERROR;
    }
    return m_protocolManager->adsd3500_write_cmd(cmd, data, usDelay);
}

Status Adsd3500CommandImpl::readPayload(uint8_t *payload,
                                        uint32_t payloadSize) {
    if (!m_protocolManager) {
        return Status::GENERIC_ERROR;
    }
    return m_protocolManager->adsd3500_read_payload(payload, payloadSize);
}

Status Adsd3500CommandImpl::writePayload(const uint8_t *payload,
                                         uint32_t payloadSize) {
    if (!m_protocolManager) {
        return Status::GENERIC_ERROR;
    }
    return m_protocolManager->adsd3500_write_payload(
        const_cast<uint8_t *>(payload), payloadSize);
}

int Adsd3500CommandImpl::getChipStatus() const { return m_chipStatus; }

int Adsd3500CommandImpl::getImagerStatus() const { return m_imagerStatus; }

} // namespace aditof
