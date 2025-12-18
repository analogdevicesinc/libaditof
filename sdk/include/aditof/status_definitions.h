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
#ifndef STATUS_DEFINITIONS_H
#define STATUS_DEFINITIONS_H

#include "sdk_exports.h"

#include <ostream>

/**
 * @brief Namespace aditof
 */
namespace aditof {

/**
 * @enum Status
 * @brief Status of any operation that the TOF sdk performs.
 */
enum class Status {
    OK,                  //!< Success
    BUSY,                //!< Device or resource is busy
    UNREACHABLE,         //!< Device or resource is unreachable
    INVALID_ARGUMENT,    //!< Invalid arguments provided
    UNAVAILABLE,         //!< The requested action or resource is unavailable
    INSUFFICIENT_MEMORY, //!< Insufficient memory or buffer size
    GENERIC_ERROR //!< An error occured but there are no details available.
};

/**
 * @enum Adsd3500Status
 * @brief Status of the ADSD3500 sensor.
 */
enum class Adsd3500Status {
    OK,                       //!< Success
    INVALID_MODE,             //!< Invalid mode has been set
    INVALID_JBLF_FILTER_SIZE, //!< Invalid JBLF filter size has been set
    UNSUPPORTED_COMMAND,      //!< The command is not supported by the ADSD3500
    INVALID_MEMORY_REGION,    //!< Invalid memory region
    INVALID_FIRMWARE_CRC,     //!< Invalid firmware CRC
    INVALID_IMAGER,           //!< Invalid imager
    INVALID_CCB,              //!< Invalid CCB
    FLASH_HEADER_PARSE_ERROR, //!< Flash header parse error
    FLASH_FILE_PARSE_ERROR,   //!< Flash file parse error
    SPIM_ERROR,               //!< SPIM error
    INVALID_CHIPID,           //!< Invalid chip ID
    IMAGER_COMMUNICATION_ERROR, //!< Imager communication error
    IMAGER_BOOT_FAILURE,        //!< Imager boot failure
    FIRMWARE_UPDATE_COMPLETE,   //!< The firmware update action has completed
    NVM_WRITE_COMPLETE,         //!< The write action to the NVM has completed
    IMAGER_ERROR,               //!< Imager error
    TIMEOUT_ERROR,              //!< Timeout error
    DYNAMIC_MODE_SWITCHING_NOT_ENABLED, //!< Dynamic mode switching is not enabled
    INVALID_DYNAMIC_MODE_COMPOSITIONS, //!< Invalid dynamic mode compositions
    INVALID_PHASE_INVALID_VALUE,       //!< Invalid phase invalid value
    CCB_WRITE_COMPLETE,                //!< CCB write completed.
    INVALID_CCB_WRITE_CRC,             //!< Invalid ccb write crc
    CFG_WRITE_COMPLETE,                //!< CFG write completed
    INVALID_CFG_WRITE_CRC,             //!< Invalid cfg write crc
    INIT_FW_WRITE_COMPLETE,            //!< Init_Fw write completed
    INVALID_INIT_FW_WRITE_CRC,         //!< Invalid Init_Fw write completed
    INVALID_BIN_SIZE,                  //!< Invalid binary size
    ACK_ERROR,                         //!< Acknowladgement error
    FLASH_STATUS_CHUNK_ALREADY_FOUND,  //!< Flash status chunck already found
    INVALID_INI_UPDATE_IN_PCM_MODE,    //!< Invalid INI update in PCM mode
    UNSUPPORTED_MODE_INI_READ,         //!< Unsupported mode INI read
    IMAGER_STREAM_OFF,                 //!< Stream off from imager
    UNKNOWN_ERROR_ID                   //!< Unknown ID read from ADSD3500
};

/**
 * @brief operator << which make possible to print out items from aditof::Status enum
 *
 * @param os - output streamm
 * @param status - an item of type aditof::Status
 * @return std::ostream&
 */
SDK_API std::ostream &operator<<(std::ostream &os, aditof::Status status);

/**
 * @brief operator << which make possible to print out items from aditof::Adsd3500Status enum
 *
 * @param os - output streamm
 * @param status - an item of type aditof::Adsd3500Status
 * @return std::ostream&
 */
SDK_API std::ostream &operator<<(std::ostream &os,
                                 aditof::Adsd3500Status status);

} // namespace aditof

#endif // STATUS_DEFINITIONS_H
