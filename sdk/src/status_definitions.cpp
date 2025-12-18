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
#include <aditof/status_definitions.h>

namespace aditof {

std::ostream &operator<<(std::ostream &os, aditof::Status status) {
    switch (status) {
    case aditof::Status::OK:
        os << "Status::OK";
        break;
    case aditof::Status::BUSY:
        os << "Status::BUSY";
        break;
    case aditof::Status::UNREACHABLE:
        os << "Status::UNREACHABLE";
        break;
    case aditof::Status::INVALID_ARGUMENT:
        os << "Status::INVALID_ARGUMENT";
        break;
    case aditof::Status::UNAVAILABLE:
        os << "Status::UNAVAILABLE";
        break;
    case aditof::Status::INSUFFICIENT_MEMORY:
        os << "Status::INSUFFICIENT_MEMORY";
        break;
    case aditof::Status::GENERIC_ERROR:
        os << "Status::GENERIC_ERROR";
        break;
    default:
        os.setstate(std::ios_base::failbit);
    }
    return os;
}

std::ostream &operator<<(std::ostream &os, aditof::Adsd3500Status status) {
    switch (status) {
    case aditof::Adsd3500Status::OK:
        os << "Adsd3500Status::OK";
        break;
    case aditof::Adsd3500Status::INVALID_MODE:
        os << "Adsd3500Status::INVALID_MODE";
        break;
    case aditof::Adsd3500Status::INVALID_JBLF_FILTER_SIZE:
        os << "Adsd3500Status::INVALID_JBLF_FILTER_SIZE";
        break;
    case aditof::Adsd3500Status::UNSUPPORTED_COMMAND:
        os << "Adsd3500Status::UNSUPPORTED_COMMAND";
        break;
    case aditof::Adsd3500Status::INVALID_MEMORY_REGION:
        os << "Adsd3500Status::INVALID_MEMORY_REGION";
        break;
    case aditof::Adsd3500Status::INVALID_FIRMWARE_CRC:
        os << "Adsd3500Status::INVALID_FIRMWARE_CRC";
        break;
    case aditof::Adsd3500Status::INVALID_IMAGER:
        os << "Adsd3500Status::INVALID_IMAGER";
        break;
    case aditof::Adsd3500Status::INVALID_CCB:
        os << "Adsd3500Status::INVALID_CCB";
        break;
    case aditof::Adsd3500Status::FLASH_HEADER_PARSE_ERROR:
        os << "Adsd3500Status::FLASH_HEADER_PARSE_ERROR";
        break;
    case aditof::Adsd3500Status::FLASH_FILE_PARSE_ERROR:
        os << "Adsd3500Status::FLASH_FILE_PARSE_ERROR";
        break;
    case aditof::Adsd3500Status::SPIM_ERROR:
        os << "Adsd3500Status::SPIM_ERROR";
        break;
    case aditof::Adsd3500Status::INVALID_CHIPID:
        os << "Adsd3500Status::INVALID_CHIPID";
        break;
    case aditof::Adsd3500Status::IMAGER_COMMUNICATION_ERROR:
        os << "Adsd3500Status::IMAGER_COMMUNICATION_ERROR";
        break;
    case aditof::Adsd3500Status::IMAGER_BOOT_FAILURE:
        os << "Adsd3500Status::IMAGER_BOOT_FAILURE";
        break;
    case aditof::Adsd3500Status::FIRMWARE_UPDATE_COMPLETE:
        os << "Adsd3500Status::FIRMWARE_UPDATE_COMPLETE";
        break;
    case aditof::Adsd3500Status::NVM_WRITE_COMPLETE:
        os << "Adsd3500Status::NVM_WRITE_COMPLETE";
        break;
    case aditof::Adsd3500Status::IMAGER_ERROR:
        os << "Adsd3500Status::IMAGER_ERROR";
        break;
    case Adsd3500Status::TIMEOUT_ERROR:
        os << "Adsd3500Status::TIMEOUT_ERROR";
        break;
    case Adsd3500Status::DYNAMIC_MODE_SWITCHING_NOT_ENABLED:
        os << "Adsd3500Status::DYNAMIC_MODE_SWITCHING_NOT_ENABLED";
        break;
    case Adsd3500Status::INVALID_DYNAMIC_MODE_COMPOSITIONS:
        os << "Adsd3500Status::INVALID_DYNAMIC_MODE_COMPOSITIONS";
        break;
    case Adsd3500Status::INVALID_PHASE_INVALID_VALUE:
        os << "Adsd3500Status::INVALID_PHASE_INVALID_VALUE";
        break;
    case Adsd3500Status::CCB_WRITE_COMPLETE:
        os << "Adsd3500Status::CCB_WRITE_COMPLETE";
        break;
    case Adsd3500Status::INVALID_CCB_WRITE_CRC:
        os << "Adsd3500Status::INVALID_CCB_WRITE_CRC";
        break;
    case Adsd3500Status::CFG_WRITE_COMPLETE:
        os << "Adsd3500Status::CFG_WRITE_COMPLETE";
        break;
    case Adsd3500Status::INVALID_CFG_WRITE_CRC:
        os << "Adsd3500Status::INVALID_CFG_WRITE_CRC";
        break;
    case Adsd3500Status::INIT_FW_WRITE_COMPLETE:
        os << "Adsd3500Status::INIT_FW_WRITE_COMPLETE";
        break;
    case Adsd3500Status::INVALID_INIT_FW_WRITE_CRC:
        os << "Adsd3500Status::INVALID_INIT_FW_WRITE_CRC";
        break;
    case Adsd3500Status::INVALID_BIN_SIZE:
        os << "Adsd3500Status::INVALID_BIN_SIZE";
        break;
    case Adsd3500Status::ACK_ERROR:
        os << "Adsd3500Status::ACK_ERROR";
        break;
    case Adsd3500Status::FLASH_STATUS_CHUNK_ALREADY_FOUND:
        os << "Adsd3500Status::FLASH_STATUS_CHUNK_ALREADY_FOUND";
        break;
    case Adsd3500Status::INVALID_INI_UPDATE_IN_PCM_MODE:
        os << "Adsd3500Status::INVALID_INI_UPDATE_IN_PCM_MODE";
        break;
    case Adsd3500Status::UNSUPPORTED_MODE_INI_READ:
        os << "Adsd3500Status::UNSUPPORTED_MODE_INI_READ";
        break;
    case Adsd3500Status::IMAGER_STREAM_OFF:
        os << "Adsd3500Status::IMAGER_STREAM_OFF";
        break;
    case aditof::Adsd3500Status::UNKNOWN_ERROR_ID:
        os << "Adsd3500Status::UNKNOWN_ERROR_ID";
        break;
    default:
        os.setstate(std::ios_base::failbit);
    }
    return os;
}

} // namespace aditof
