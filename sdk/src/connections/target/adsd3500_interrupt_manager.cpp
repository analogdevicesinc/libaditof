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
#include "adsd3500_interrupt_manager.h"
#include "adsd3500_interrupt_notifier.h"
#include "adsd3500_protocol_manager.h"
#include <aditof/log.h>
#include <unistd.h>

#define ADSD3500_STATUS_READ_DELAY_US 2000

Adsd3500InterruptManager::Adsd3500InterruptManager(
    Adsd3500ProtocolManager *protocolManager, int &chipStatus,
    int &imagerStatus, bool &interruptAvailable,
    std::unordered_map<void *, aditof::SensorInterruptCallback>
        &interruptCallbackMap)
    : m_protocolManager(protocolManager), m_chipStatus(chipStatus),
      m_imagerStatus(imagerStatus), m_interruptAvailable(interruptAvailable),
      m_interruptCallbackMap(interruptCallbackMap) {}

aditof::Status Adsd3500InterruptManager::adsd3500_register_interrupt_callback(
    aditof::SensorInterruptCallback &cb) {
    if (Adsd3500InterruptNotifier::getInstance().interruptsAvailable()) {
        m_interruptCallbackMap.insert({&cb, cb});
    } else {
        return aditof::Status::UNAVAILABLE;
    }

    return aditof::Status::OK;
}

aditof::Status Adsd3500InterruptManager::adsd3500_unregister_interrupt_callback(
    aditof::SensorInterruptCallback &cb) {

    m_interruptCallbackMap.erase(&cb);

    return aditof::Status::OK;
}

aditof::Status
Adsd3500InterruptManager::adsd3500InterruptHandler(int signalValue) {
    uint16_t statusRegister;
    aditof::Status status = aditof::Status::OK;

    usleep(ADSD3500_STATUS_READ_DELAY_US);

    status = m_protocolManager->adsd3500_read_cmd(0x0020, &statusRegister);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to read status register!";
        return status;
    }

    aditof::Adsd3500Status adsd3500Status =
        convertIdToAdsd3500Status(statusRegister);
    DLOG(INFO) << "statusRegister:" << statusRegister << "(" << adsd3500Status
               << ")";

    m_chipStatus = statusRegister;

    if (adsd3500Status == aditof::Adsd3500Status::IMAGER_ERROR) {
        status = m_protocolManager->adsd3500_read_cmd(0x0038, &statusRegister);
        if (status != aditof::Status::OK) {
            LOG(ERROR) << "Failed to read imager status register!";
            return status;
        }

        m_imagerStatus = statusRegister;
        LOG(ERROR) << "Imager error detected. Error code: " << statusRegister;
    }

    for (const auto &callback : m_interruptCallbackMap) {
        callback.second(adsd3500Status);
    }

    if (status == aditof::Status::OK) {
        m_interruptAvailable = true;
    }

    return status;
}

aditof::Adsd3500Status
Adsd3500InterruptManager::convertIdToAdsd3500Status(int status) {
    using namespace aditof;

    switch (status) {
    case 0:
        return Adsd3500Status::OK;

    case 1:
        return Adsd3500Status::INVALID_MODE;

    case 2:
        return Adsd3500Status::INVALID_JBLF_FILTER_SIZE;

    case 3:
        return Adsd3500Status::UNSUPPORTED_COMMAND;

    case 4:
        return Adsd3500Status::INVALID_MEMORY_REGION;

    case 5:
        return Adsd3500Status::INVALID_FIRMWARE_CRC;

    case 6:
        return Adsd3500Status::INVALID_IMAGER;

    case 7:
        return Adsd3500Status::INVALID_CCB;

    case 8:
        return Adsd3500Status::FLASH_HEADER_PARSE_ERROR;

    case 9:
        return Adsd3500Status::FLASH_FILE_PARSE_ERROR;

    case 10:
        return Adsd3500Status::SPIM_ERROR;

    case 11:
        return Adsd3500Status::INVALID_CHIPID;

    case 12:
        return Adsd3500Status::IMAGER_COMMUNICATION_ERROR;

    case 13:
        return Adsd3500Status::IMAGER_BOOT_FAILURE;

    case 14:
        return Adsd3500Status::FIRMWARE_UPDATE_COMPLETE;

    case 15:
        return Adsd3500Status::NVM_WRITE_COMPLETE;

    case 16:
        return Adsd3500Status::IMAGER_ERROR;

    case 17:
        return Adsd3500Status::TIMEOUT_ERROR;

    case 19:
        return Adsd3500Status::DYNAMIC_MODE_SWITCHING_NOT_ENABLED;

    case 20:
        return Adsd3500Status::INVALID_DYNAMIC_MODE_COMPOSITIONS;

    case 21:
        return Adsd3500Status::INVALID_PHASE_INVALID_VALUE;

    case 22:
        return Adsd3500Status::CCB_WRITE_COMPLETE;

    case 23:
        return Adsd3500Status::INVALID_CCB_WRITE_CRC;

    case 24:
        return Adsd3500Status::CFG_WRITE_COMPLETE;

    case 25:
        return Adsd3500Status::INVALID_CFG_WRITE_CRC;

    case 26:
        return Adsd3500Status::INIT_FW_WRITE_COMPLETE;

    case 27:
        return Adsd3500Status::INVALID_INIT_FW_WRITE_CRC;

    case 28:
        return Adsd3500Status::INVALID_BIN_SIZE;

    case 29:
        return Adsd3500Status::ACK_ERROR;

    case 30:
        return Adsd3500Status::FLASH_STATUS_CHUNK_ALREADY_FOUND;

    case 34:
        return Adsd3500Status::INVALID_INI_UPDATE_IN_PCM_MODE;

    case 35:
        return Adsd3500Status::UNSUPPORTED_MODE_INI_READ;

    case 41:
        return Adsd3500Status::IMAGER_STREAM_OFF;

    default: {
        LOG(ERROR) << "Unknown ID: " << status;
        return Adsd3500Status::UNKNOWN_ERROR_ID;
    }
    }
}
