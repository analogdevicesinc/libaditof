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
#include "adsd3500_sensor.h"
#include "../cameras/itof-camera/adsd3500_registers.h"
#include "aditof/frame_operations.h"
#include "aditof/utils.h"
#include "adsd3500_interrupt_notifier.h"
#include "gpio.h"
#include "platform/platform_impl.h"
#include "sensor-tables/device_parameters.h"
#include "utils_ini.h"

#include "tofi/tofi_config.h"
#include <aditof/log.h>
#include <algorithm>
#include <arm_neon.h>
#include <array>
#include <cmath>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <fstream>
#include <linux/videodev2.h>
#include <signal.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>

#define MAX_SUBFRAMES_COUNT                                                    \
    10 // maximum number of subframes that are used to create a full frame (maximum total_captures of all modes)
#define EXTRA_BUFFERS_COUNT                                                    \
    3 // how many extra buffers are sent to the driver in addition to the total_captures of a mode

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define CTRL_PACKET_SIZE 65537
#define CTRL_SET_FRAME_RATE (0x9a200b)
#define V4L2_CID_AD_DEV_CHIP_CONFIG                                            \
    (aditof::platform::Platform::getInstance().getV4L2ChipConfigControlId())
#define CTRL_SET_MODE                                                          \
    (aditof::platform::Platform::getInstance().getV4L2ModeControlId())
#define CTRL_AB_AVG                                                            \
    (aditof::platform::Platform::getInstance().getV4L2AbAvgControlId())
#define CTRL_DEPTH_EN                                                          \
    (aditof::platform::Platform::getInstance().getV4L2DepthEnControlId())
#define CTRL_PHASE_DEPTH_BITS                                                  \
    (aditof::platform::Platform::getInstance().getV4L2PhaseDepthBitsControlId())
#define CTRL_AB_BITS                                                           \
    (aditof::platform::Platform::getInstance().getV4L2AbBitsControlId())
#define CTRL_CONFIDENCE_BITS                                                   \
    (aditof::platform::Platform::getInstance().getV4L2ConfidenceBitsControlId())

#define ADSD3500_CTRL_PACKET_SIZE 4099

// ADSD3500 Burst Protocol Packet Structure Offsets
// Burst command packet: [Header(3B)][Marker(1B)][Cmd(3B)][Pad(4B)][Checksum(4B)][Payload...]
#define ADSD3500_BURST_CMD_HEADER_SIZE                                         \
    15 // Size of command header before payload data
// Burst response packet: [Header(3B)][Payload...]
#define ADSD3500_BURST_RESPONSE_HEADER_SIZE                                    \
    3 // Size of response header before payload data

#define NR_OF_MODES_FROM_CCB 10
#define SIZE_OF_MODES_FROM_CCB 256

// Timing delays in microseconds
#define ADSD3500_STATUS_READ_DELAY_US                                          \
    2000 // 2ms - delay before reading status after interrupt
#define ADSD3500_BURST_CMD_SHORT_DELAY_US                                      \
    1000 // 1ms - delay after burst command 0x13
#define ADSD3500_BURST_CMD_LONG_DELAY_US                                       \
    5000 // 5ms - delay after burst command 0x19
#define ADSD3500_PAYLOAD_READ_DELAY_US                                         \
    30000 // 30ms - delay before reading payload
#define ADSD3500_PAYLOAD_WRITE_DELAY_US                                        \
    100000 // 100ms - delay after writing payload

struct CalibrationData {
    std::string mode;
    float gain;
    float offset;
    uint16_t *cache;
};

struct ConfigurationData {
    uint16_t id;
    uint16_t ver;
    uint32_t size;
    uint16_t burst_layout;
    uint16_t burst_num;
    uint16_t burst_setup[4];
    uint16_t start_address;
    uint16_t rsvd;
    uint32_t values;
};

struct Adsd3500Sensor::ImplData {
    uint8_t numVideoDevs; /**< Number of V4L2 video devices for this sensor */
    struct VideoDev *videoDevs; /**< Array of V4L2 video device descriptors */
    aditof::DepthSensorModeDetails
        modeDetails; /**< Current frame mode details and capabilities */
    std::unordered_map<std::string, __u32>
        controlsCommands; /**< Map of V4L2 control command names to IDs */
    SensorImagerType imagerType; /**< Type of ADTF3175D imager (detector) */
    CCBVersion ccbVersion; /**< CCB (Camera Control Board) firmware version */
    std::string fw_ver;    /**< ISP firmware version string */
    std::array<uint8_t, ADSD3500_CTRL_PACKET_SIZE>
        ctrlBuf; /**< Buffer for ISP control commands and responses */

    ImplData()
        : numVideoDevs(1),
          videoDevs(nullptr), modeDetails{0, {}, 0, 0, 0, 0, 0, 0, 0, 0, {}} {
        ccbVersion = CCBVersion::CCB_UNKNOWN;
        imagerType = SensorImagerType::IMAGER_UNKNOWN;
    }
};

/**
 * @brief Wrapper for ioctl that retries on EINTR with validation.
 *
 * Repeatedly calls ioctl until it succeeds or fails with an error other than EINTR.
 * Validates the file descriptor before attempting ioctl calls and retries up to 3 times.
 *
 * @param[in] fh File descriptor
 * @param[in] request ioctl request code
 * @param[in,out] arg Pointer to ioctl argument structure
 *
 * @return ioctl result: 0 on success, -1 on error (with errno set)
 */
static int xioctl(int fh, unsigned int request, void *arg) {
    int r;
    int tries = 3;

    // Validate file handle before calling ioctl
    if (fh < 0) {
        LOG(ERROR) << "xioctl called with invalid file descriptor: " << fh;
        errno = EBADF;
        return -1;
    }

    // Note: Argument validation removed - the kernel's ioctl properly validates
    // arguments and returns appropriate error codes (EFAULT, EINVAL, etc.)
    // Some V4L2 ioctls legitimately accept NULL pointers for certain operations

    do {
        r = ioctl(fh, request, arg);
    } while (--tries > 0 && r == -1 && EINTR == errno);

    if (r == -1) {
        LOG(WARNING) << "xioctl failed: fd=" << fh << " request=0x" << std::hex
                     << request << " errno=" << std::dec << errno << " ("
                     << strerror(errno) << ")"
                     << " after " << (4 - tries) << " attempts";
    }

    return r;
}

/**
 * @brief Constructs an Adsd3500Sensor object.
 *
 * Initializes the ADSD3500 depth sensor with driver paths, creates implementation data,
 * sets up control parameters, and instantiates the buffer processor. Configures sensor
 * name, connection type, and available controls for depth, AB, confidence settings.
 *
 * @param[in] driverPath Path to the main V4L2 video device driver
 * @param[in] driverSubPath Path to the sub-device driver
 * @param[in] captureDev Path to the capture device
 */
Adsd3500Sensor::Adsd3500Sensor(const std::string &driverPath,
                               const std::string &driverSubPath,
                               const std::string &captureDev)
    : m_driverPath(driverPath), m_driverSubPath(driverSubPath),
      m_captureDev(captureDev), m_implData(new Adsd3500Sensor::ImplData),
      m_firstRun(true), m_adsd3500Queried(false), m_depthComputeOnTarget(true),
      m_chipStatus(0), m_imagerStatus(0), m_isOpen(false), m_bitsInAB(0),
      m_bitsInConf(0), m_chipId(0) {
    m_sensorName = "adsd3500";
    m_interruptAvailable = false;
    m_sensorDetails.connectionType = aditof::ConnectionType::ON_TARGET;
    m_sensorDetails.id = driverPath;
    m_sensorConfiguration = "standard";

    // Define the controls that this sensor has available
    m_controls.emplace("abAveraging", "0");
    m_controls.emplace("depthEnable", "0");
    m_controls.emplace("phaseDepthBits", "0");
    m_controls.emplace("abBits", "0");
    m_controls.emplace("confidenceBits", "0");
    m_controls.emplace("fps", "0");
    m_controls.emplace("imagerType", "");
    m_controls.emplace("inputFormat", "");
    m_controls.emplace("netlinktest", "0");
    m_controls.emplace("depthComputeOpenSource", "0");
    m_controls.emplace("disableCCBM", "0");
    m_controls.emplace("availableCCBM", "0");
    m_controls.emplace("lensScatterCompensationEnabled", "0");
    m_controls.emplace("enableRotation", "0");
    m_controls.emplace("targetModeNumber", "-1");

    // Define the commands that correspond to the sensor controls
    m_implData->controlsCommands["abAveraging"] = CTRL_AB_AVG;
    m_implData->controlsCommands["depthEnable"] = CTRL_DEPTH_EN;
    m_implData->controlsCommands["phaseDepthBits"] = CTRL_PHASE_DEPTH_BITS;
    m_implData->controlsCommands["abBits"] = CTRL_AB_BITS;
    m_implData->controlsCommands["confidenceBits"] = CTRL_CONFIDENCE_BITS;

    m_bufferProcessor = new BufferProcessor();
}

/**
 * @brief Destructor for Adsd3500Sensor.
 *
 * Stops streaming if active, unmaps video buffers, closes file descriptors, and
 * deallocates video device structures and buffer processor. Ensures proper cleanup
 * of all allocated resources.
 */
Adsd3500Sensor::~Adsd3500Sensor() {
    struct VideoDev *dev = nullptr;

    // Check if videoDevs array is valid before accessing
    if (m_implData == nullptr || m_implData->videoDevs == nullptr) {
        delete m_bufferProcessor;
        return;
    }

    // Extract count to make analyzer understand m_implData is definitely non-null
    unsigned int numVideoDevs = m_implData->numVideoDevs;

    for (unsigned int i = 0; i < numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];
        if (dev->started) {
            // NOLINTNEXTLINE(clang-analyzer-optin.cplusplus.VirtualCall)
            Adsd3500Sensor::stop();
        }
    }

    for (unsigned int i = 0; i < numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];

        if (dev->videoBuffers) {
            for (unsigned int j = 0; j < dev->nVideoBuffers; j++) {
                if (munmap(dev->videoBuffers[j].start,
                           dev->videoBuffers[j].length) == -1) {
                    LOG(WARNING)
                        << "munmap error "
                        << "errno: " << errno << " error: " << strerror(errno);
                }
            }
            free(dev->videoBuffers);
            dev->videoBuffers = nullptr;
        }

        if (dev->fd != -1) {
            if (close(dev->fd) == -1) {
                LOG(WARNING)
                    << "close m_implData->fd error "
                    << "errno: " << errno << " error: " << strerror(errno);
            }
        }

        if (dev->sfd != -1) {
            if (close(dev->sfd) == -1) {
                LOG(WARNING)
                    << "close m_implData->sfd error "
                    << "errno: " << errno << " error: " << strerror(errno);
            }
        }
    }

    // Always clean up videoDevs regardless of m_isOpen state to prevent memory leaks
    if (m_implData != nullptr && m_implData->videoDevs) {
        delete[] m_implData->videoDevs;
        m_implData->videoDevs = nullptr;
    }
    delete m_bufferProcessor;
}

/**
 * @brief Opens the ADSD3500 sensor and initializes V4L2 devices.
 *
 * Subscribes to interrupt notifications, opens V4L2 video devices, queries capabilities,
 * checks for subdevice support, allocates and maps video buffers for MMAP, initializes
 * buffer processor, and queries ADSD3500 chip information. On first run, enables interrupts.
 *
 * @return Status::OK on success, Status::GENERIC_ERROR on failure
 */
aditof::Status Adsd3500Sensor::open() {
    using namespace aditof;
    Status status = Status::OK;

    // Subscribe to ADSD3500 interrupts
    if (m_firstRun) {
        if (Adsd3500InterruptNotifier::getInstance().interruptsAvailable()) {
            std::weak_ptr<Adsd3500Sensor> wptr = shared_from_this();
            Adsd3500InterruptNotifier::getInstance().subscribeSensor(wptr);
        }
    }

    LOG(INFO) << "Opening device";

    struct stat st;
    struct v4l2_capability cap;
    struct VideoDev *dev;

    const char *devName, *subDevName, *cardName;

    std::vector<std::string> driverPaths;
    Utils::splitIntoTokens(m_driverPath, '|', driverPaths);

    std::vector<std::string> driverSubPaths;
    Utils::splitIntoTokens(m_driverSubPath, '|', driverSubPaths);

    std::vector<std::string> cards;
    std::string captureDeviceName(m_captureDev);
    Utils::splitIntoTokens(captureDeviceName, '|', cards);

    LOG(INFO) << "Looking for the following cards:";
    for (const auto &card : cards) {
        LOG(INFO) << card;
    }

    m_implData->numVideoDevs = driverSubPaths.size();

    // Validate all configuration vectors have matching sizes
    if (driverPaths.size() != driverSubPaths.size() ||
        cards.size() != driverSubPaths.size()) {
        LOG(ERROR)
            << "Driver paths, sub-paths, and card names must have same size. "
            << "Paths: " << driverPaths.size()
            << ", SubPaths: " << driverSubPaths.size()
            << ", Cards: " << cards.size();
        return Status::GENERIC_ERROR;
    }

    // Close file descriptors and delete any existing videoDevs to prevent leaks on repeated open() calls
    if (m_implData->videoDevs) {
        for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
            struct VideoDev *oldDev = &m_implData->videoDevs[i];

            // Close file descriptors before deleting
            if (oldDev->fd != -1) {
                close(oldDev->fd);
                oldDev->fd = -1;
            }
            if (oldDev->sfd != -1) {
                close(oldDev->sfd);
                oldDev->sfd = -1;
            }

            // Free video buffers if still allocated
            if (oldDev->videoBuffers) {
                for (unsigned int j = 0; j < oldDev->nVideoBuffers; j++) {
                    if (oldDev->videoBuffers[j].start &&
                        oldDev->videoBuffers[j].start != MAP_FAILED) {
                        munmap(oldDev->videoBuffers[j].start,
                               oldDev->videoBuffers[j].length);
                    }
                }
                free(oldDev->videoBuffers);
                oldDev->videoBuffers = nullptr;
            }
        }
        delete[] m_implData->videoDevs;
        m_implData->videoDevs = nullptr;
    }

    m_implData->videoDevs =
        new (std::nothrow) VideoDev[m_implData->numVideoDevs];
    if (!m_implData->videoDevs) {
        LOG(ERROR) << "Failed to allocate memory for "
                   << m_implData->numVideoDevs << " video devices";
        return Status::GENERIC_ERROR;
    }

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        devName = driverPaths.at(i).c_str();
        subDevName = driverSubPaths.at(i).c_str();
        cardName = cards.at(i).c_str();
        dev = &m_implData->videoDevs[i];

        LOG(INFO) << "device: " << devName << "\tsubdevice: " << subDevName;

        /* Open V4L2 device */
        if (stat(devName, &st) == -1) {
            LOG(WARNING) << "Cannot identify " << devName << "errno: " << errno
                         << "error: " << strerror(errno);
            goto cleanup_on_open_error;
        }

        if (!S_ISCHR(st.st_mode)) {
            LOG(WARNING) << devName << " is not a valid device";
            goto cleanup_on_open_error;
        }

        dev->fd = ::open(devName, O_RDWR | O_NONBLOCK, 0);
        if (dev->fd == -1) {
            LOG(WARNING) << "Cannot open " << devName << "errno: " << errno
                         << "error: " << strerror(errno);
            goto cleanup_on_open_error;
        }

        // Validate file descriptor is valid before using
        if (dev->fd < 0) {
            LOG(ERROR) << "Invalid file descriptor for " << devName;
            goto cleanup_on_open_error;
        }

        if (xioctl(dev->fd, VIDIOC_QUERYCAP, &cap) == -1) {
            LOG(WARNING) << devName << " VIDIOC_QUERYCAP error";
            goto cleanup_on_open_error;
        }

        if (strncmp((char *)cap.card, cardName, strlen(cardName))) {
            LOG(WARNING) << "CAPTURE Device " << cap.card;
            LOG(WARNING) << "Read " << cardName;
            goto cleanup_on_open_error;
        }

        if (!(cap.capabilities &
              (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE))) {
            LOG(WARNING) << devName << " is not a video capture device";
            goto cleanup_on_open_error;
        }

        if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
            dev->videoBuffersType = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        } else {
            dev->videoBuffersType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        }

        if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
            LOG(WARNING) << devName << " does not support streaming i/o";
            goto cleanup_on_open_error;
        }

        /* Open V4L2 subdevice */
        if (stat(subDevName, &st) == -1) {
            LOG(WARNING) << "Cannot identify " << subDevName
                         << " errno: " << errno
                         << " error: " << strerror(errno);
            goto cleanup_on_open_error;
        }

        if (!S_ISCHR(st.st_mode)) {
            LOG(WARNING) << subDevName << " is not a valid device";
            goto cleanup_on_open_error;
        }

        dev->sfd = ::open(subDevName, O_RDWR | O_NONBLOCK);
        if (dev->sfd == -1) {
            LOG(WARNING) << "Cannot open " << subDevName << " errno: " << errno
                         << " error: " << strerror(errno);
            goto cleanup_on_open_error;
        }

        // Validate file descriptor is valid before using
        if (dev->sfd < 0) {
            LOG(ERROR) << "Invalid file descriptor for " << subDevName;
            goto cleanup_on_open_error;
        }

        // Initialize protocol manager after video devices are opened (required for reset)
        if (!m_protocolManager) {
            m_protocolManager = std::make_unique<Adsd3500ProtocolManager>(
                m_implData->videoDevs, m_implData->ctrlBuf, m_adsd3500_mutex);
        }

        // Initialize interrupt manager BEFORE reset (required for callback registration)
        if (!m_interruptManager) {
            m_interruptManager = std::make_unique<Adsd3500InterruptManager>(
                m_protocolManager.get(), m_chipStatus, m_imagerStatus,
                m_interruptAvailable, m_interruptCallbackMap);
        }

        //Check chip status and reset if there are any errors.
        if (m_firstRun) {
            aditof::Status chipIDStatus = aditof::Status::GENERIC_ERROR;
            aditof::Status dealiasStatus = aditof::Status::GENERIC_ERROR;
            aditof::Status adsd3500StateStatus = aditof::Status::GENERIC_ERROR;
            uint16_t chipID;
            uint16_t adsd3500State;
            uint8_t dealiasCheck[32] = {0};
            dealiasCheck[0] = 1;

            // Limit reset attempts to prevent excessive startup delays
            const int MAX_RESET_ATTEMPTS = 3;

            for (int i = 0; i < MAX_RESET_ATTEMPTS; i++) {
                LOG(INFO) << "ADSD3500 initialization attempt " << (i + 1)
                          << " of " << MAX_RESET_ATTEMPTS;
                adsd3500_reset();

                adsd3500StateStatus =
                    adsd3500_read_cmd(ADSD3500_REG_CHIP_STATUS, &adsd3500State);
                if (adsd3500StateStatus != Status::OK) {
                    LOG(INFO) << "Could not read the status register";
                    continue;
                }

                if ((adsd3500State != 0x0) && (adsd3500State != 0x29)) {
                    LOG(INFO) << "ADSD3500 is not in good state (status: 0x"
                              << std::hex << adsd3500State << std::dec << ")";
                    continue;
                }

#ifdef DUAL
                chipIDStatus = adsd3500_read_cmd(ADSD3500_REG_CHIP_ID_EXT,
                                                 &chipID, 110 * 1000);
#else
                chipIDStatus = adsd3500_read_cmd(ADSD3500_REG_CHIP_ID, &chipID);
#endif
                if (chipIDStatus != Status::OK) {
                    LOG(INFO) << "Could not read chip ID";
                    continue;
                }

                m_chipId = chipID;

                dealiasStatus =
                    adsd3500_read_payload_cmd(0x02, dealiasCheck, 32);
                if (dealiasStatus != Status::OK) {
                    LOG(INFO) << "Could not read Dealias Parameters";
                    continue;
                }

                if (chipIDStatus == aditof::Status::OK &&
                    dealiasStatus == aditof::Status::OK) {
                    LOG(INFO) << "ADSD3500 is ready to communicate with after "
                              << (i + 1) << " attempt(s)";
                    break;
                }
            }

            if (chipIDStatus != aditof::Status::OK) {
                LOG(ERROR) << "Cannot read chip id after " << MAX_RESET_ATTEMPTS
                           << " attempts! Latest ADSD3500 programming might "
                              "not be successful";
                goto cleanup_on_open_error;
            }

            if (dealiasStatus != aditof::Status::OK) {
                LOG(ERROR) << "Cannot read dealias parameters after "
                           << MAX_RESET_ATTEMPTS
                           << " attempts! Latest ADSD3500 programming might "
                              "not be successful";
                goto cleanup_on_open_error;
            }

            m_firstRun = false;
        }
    }

    // Protocol manager and interrupt manager already created above before reset

    // Initialize chip config manager after protocol manager
    if (!m_chipConfigManager) {
        m_chipConfigManager = std::make_unique<Adsd3500ChipConfigManager>(
            *m_protocolManager, m_modeSelector, m_implData->imagerType,
            m_implData->ccbVersion, m_implData->fw_ver, m_controls, m_chipId);
    }

    if (!m_adsd3500Queried) {
        status = queryAdsd3500();
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to query ADSD3500 sensor!";
            goto cleanup_on_open_error;
        }
        m_adsd3500Queried = true;
    }

    status = m_bufferProcessor->setInputDevice(m_implData->videoDevs);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to set input video device!";
        return status;
    }

    status = m_bufferProcessor->open();
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to open output video device!";
        return status;
    }

    // Buffer manager initialization (protocol manager already created above)
    if (!m_bufferManager) {
        m_bufferManager = std::make_unique<V4L2BufferManager>(
            m_implData->videoDevs, m_implData->numVideoDevs);
    }

    // Initialize INI configuration manager
    if (!m_iniConfigManager) {
        m_iniConfigManager = std::make_unique<IniConfigManager>(
            m_iniFileStructList, m_ccbmINIContent, m_ccbmEnabled,
            m_implData->imagerType);
    }

    if (status == aditof::Status::OK) {
        m_isOpen = true;
    }

    return status;

cleanup_on_open_error:
    // Close all file descriptors that were successfully opened in the loop
    for (unsigned int j = 0; j < m_implData->numVideoDevs; j++) {
        struct VideoDev *errorDev = &m_implData->videoDevs[j];
        if (errorDev->fd != -1) {
            close(errorDev->fd);
            errorDev->fd = -1;
        }
        if (errorDev->sfd != -1) {
            close(errorDev->sfd);
            errorDev->sfd = -1;
        }
    }
    return Status::GENERIC_ERROR;
}

/**
 * @brief Starts video streaming from the ADSD3500 sensor.
 *
 * Queues all pre-allocated video buffers to the driver, enables streaming with VIDIOC_STREAMON,
 * and starts the buffer processor threads for frame capture and processing.
 *
 * @return Status::OK on success, Status::BUSY if already started, Status::GENERIC_ERROR on ioctl failure
 */
aditof::Status Adsd3500Sensor::start() {
    using namespace aditof;
    Status status = Status::OK;
    struct VideoDev *dev;
    struct v4l2_buffer buf;

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];
        if (dev->started) {
            LOG(INFO) << "Device already started";
            return Status::BUSY;
        }
        LOG(INFO) << "Starting device " << i;

        for (unsigned int i = 0; i < dev->nVideoBuffers; i++) {
            CLEAR(buf);
            buf.type = dev->videoBuffersType;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            buf.m.planes = dev->planes;
            buf.length = 1;

            if (xioctl(dev->fd, VIDIOC_QBUF, &buf) == -1) {
                LOG(WARNING)
                    << "mmap error "
                    << "errno: " << errno << " error: " << strerror(errno);
                return Status::GENERIC_ERROR;
            }
        }

        if (xioctl(dev->fd, VIDIOC_STREAMON, &dev->videoBuffersType) != 0) {
            LOG(WARNING) << "VIDIOC_STREAMON error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

        dev->started = true;
    }
    m_bufferProcessor->startThreads();

    return status;
}

/**
 * @brief Stops video streaming from the ADSD3500 sensor.
 *
 * Stops buffer processor threads, disables streaming with VIDIOC_STREAMOFF, and marks
 * devices as stopped. Safe to call even if already stopped.
 *
 * @return Status::OK on success, Status::GENERIC_ERROR on ioctl failure
 */
aditof::Status Adsd3500Sensor::stop() {
    using namespace aditof;
    Status status = Status::OK;
    struct VideoDev *dev;

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];

        if (dev->started) {
            m_bufferProcessor->stopThreads();
            if (!dev->started) {
                LOG(INFO) << "Device " << i << " already stopped";
                return Status::OK;
            }
            LOG(INFO) << "Stopping device";

            if (xioctl(dev->fd, VIDIOC_STREAMOFF, &dev->videoBuffersType) !=
                0) {
                LOG(WARNING)
                    << "VIDIOC_STREAMOFF error "
                    << "errno: " << errno << " error: " << strerror(errno);
                return Status::GENERIC_ERROR;
            }

            dev->started = false;
        }
    }
    status = Adsd3500Sensor::adsd3500_getInterruptandReset();
    return status;
}

/**
 * @brief Retrieves available sensor modes from the mode selector.
 *
 * Queries the ADSD3500 mode selector for available mode indices and populates the
 * provided vector with mode numbers.
 *
 * @param[out] modes Vector to be filled with available mode numbers
 *
 * @return Status::OK on success
 */
aditof::Status Adsd3500Sensor::getAvailableModes(std::vector<uint8_t> &modes) {
    modes.clear();
    for (const auto &availableMode : m_availableModes) {
#ifndef ENABLE_PCM
        if (availableMode.isPCM)
            continue;
#endif
        modes.emplace_back(availableMode.modeNumber);
    }
    return aditof::Status::OK;
}

aditof::Status
Adsd3500Sensor::getModeDetails(const uint8_t &mode,
                               aditof::DepthSensorModeDetails &details) {
    using namespace aditof;
    Status status = Status::OK;
    for (const auto &modeDetails : m_availableModes) {
        if (modeDetails.modeNumber == mode) {
            details = modeDetails;
            break;
        }
    }
    return status;
}

/**
 * @brief Sets the sensor operating mode.
 *
 * Configures the ADSD3500 sensor with the specified mode, retrieves mode configuration
 * from the mode selector, updates V4L2 controls for bit configurations, and initializes
 * video properties and depth compute parameters for the buffer processor.
 *
 * @param[in] mode Mode number to set
 *
 * @return Status::OK on success, Status::GENERIC_ERROR on configuration failure
 */
aditof::Status Adsd3500Sensor::setMode(const uint8_t &mode) {
    aditof::DepthSensorModeDetails modeTable;
    aditof::Status status = aditof::Status::OK;

    status = m_modeSelector.setControl("mode", std::to_string(mode));
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to set control in modeSelector!";
        return status;
    }

    if (m_ccbmEnabled && m_sensorConfiguration == "standard") {
        bool modeFound = false;
        for (auto table : m_availableModes) {
            if (mode == table.modeNumber) {
                modeTable = table;
                modeFound = true;
                break;
            }
        }

        if (!modeFound) {
            LOG(ERROR) << "Mode not found in ccb!";
            return aditof::Status::INVALID_ARGUMENT;
        }

    } else {
        status = m_modeSelector.getConfigurationTable(modeTable);
        if (status != aditof::Status::OK) {
            LOG(ERROR) << "Failed to get configuration table!";
            return aditof::Status::GENERIC_ERROR;
        }
    }

    status = m_modeSelector.updateConfigurationTable(modeTable);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to update configuration table for currrent "
                      "configuration";
        return aditof::Status::GENERIC_ERROR;
    }

    // Update m_availableModes with the modified modeTable (e.g., isRawBypass flag)
    // so that getModeDetails() returns the correct details
    for (auto &availableMode : m_availableModes) {
        if (availableMode.modeNumber == mode) {
            availableMode = modeTable;
            break;
        }
    }

    status = setMode(modeTable);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to set mode for the current configuration!";
        return aditof::Status::GENERIC_ERROR;
    }

    m_implData->modeDetails = modeTable;
    // Reset target mode now that setMode has used the updated bit arrays
    m_targetModeNumber = -1;
    return aditof::Status::OK;
}

aditof::Status
Adsd3500Sensor::setMode(const aditof::DepthSensorModeDetails &type) {

    using namespace aditof;
    Status status = Status::OK;
    struct VideoDev *dev;

    // Validate m_implData and videoDevs are allocated before accessing
    if (!m_implData || !m_implData->videoDevs) {
        LOG(ERROR) << "Sensor not properly initialized, cannot set mode";
        return Status::GENERIC_ERROR;
    }

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];
        if (dev->started) {
            stop();
        }
    }

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];

        for (unsigned int i = 0; i < dev->nVideoBuffers; i++) {
            if (munmap(dev->videoBuffers[i].start,
                       dev->videoBuffers[i].length) == -1) {
                LOG(WARNING)
                    << "munmap error "
                    << "errno: " << errno << " error: " << strerror(errno);
            }
        }
        free(dev->videoBuffers);

        if (dev->fd != -1) {
            if (close(dev->fd) == -1) {
                LOG(WARNING)
                    << "close m_implData->fd error "
                    << "errno: " << errno << " error: " << strerror(errno);
            }
        }

        if (dev->sfd != -1) {
            if (close(dev->sfd) == -1) {
                LOG(WARNING)
                    << "close m_implData->sfd error "
                    << "errno: " << errno << " error: " << strerror(errno);
            }
        }
    }

    if (m_isOpen) { // open the device if it's been closed
        // free the allocated new buffer
        if (m_implData->videoDevs) {
            delete[] m_implData->videoDevs;
            m_implData->videoDevs = nullptr;
        }
        m_isOpen = false;
        status = open();
        if (status != aditof::Status::OK) {
            LOG(INFO) << "Failed to open sensor!";
            return status;
        }
    }

    m_capturesPerFrame = 1;

    // Validate m_implData and videoDevs are still valid after stop()
    if (!m_implData || !m_implData->videoDevs) {
        LOG(ERROR) << "Video devices invalid after stop";
        return Status::GENERIC_ERROR;
    }

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];

        // Validate file descriptors are still valid after stop()
        if (dev->fd == -1 || dev->sfd == -1) {
            LOG(ERROR) << "File descriptors invalid after stop (fd=" << dev->fd
                       << ", sfd=" << dev->sfd << ")";
            status = Status::GENERIC_ERROR;
            goto cleanup_on_error;
        }

        //Set mode in chip code block
        struct v4l2_requestbuffers req;
        struct v4l2_buffer buf;
        struct v4l2_format fmt;
        size_t length, offset;

        struct v4l2_control ctrl;

        memset(&ctrl, 0, sizeof(ctrl));

        ctrl.id = CTRL_SET_MODE;
        ctrl.value = type.modeNumber;

        if (xioctl(dev->sfd, VIDIOC_S_CTRL, &ctrl) == -1) {
            LOG(ERROR) << "Setting Mode error "
                       << "errno: " << errno << " error: " << strerror(errno);
            status = Status::GENERIC_ERROR;
            goto cleanup_on_error;
        }

        // Configure media pipeline via platform abstraction
        status =
            aditof::platform::Platform::getInstance().configureMediaPipeline(
                m_driverPath, type.frameWidthInBytes, type.frameHeightInBytes,
                (type.pixelFormatIndex == 1) ? 12 : 8);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to configure media pipeline";
            return status;
        }

        //End of set mode in chip

        if (type.modeNumber != m_implData->modeDetails.modeNumber &&
            dev->videoBuffers) {

            for (unsigned int j = 0; j < dev->nVideoBuffers; j++) {
                if (munmap(dev->videoBuffers[j].start,
                           dev->videoBuffers[j].length) == -1) {
                    LOG(WARNING)
                        << "munmap error "
                        << "errno: " << errno << " error: " << strerror(errno);
                    status = Status::GENERIC_ERROR;
                    goto cleanup_on_error;
                }
            }
            free(dev->videoBuffers);
            dev->videoBuffers = nullptr;
            dev->nVideoBuffers = 0;
            CLEAR(req);
            req.count = 0;
            req.type = dev->videoBuffersType;
            req.memory = V4L2_MEMORY_MMAP;

            if (xioctl(dev->fd, VIDIOC_REQBUFS, &req) == -1) {
                LOG(WARNING)
                    << "VIDIOC_REQBUFS error "
                    << "errno: " << errno << " error: " << strerror(errno);
                status = Status::GENERIC_ERROR;
                goto cleanup_on_error;
            }
        }

        __u32 pixelFormat = 0;

        if (type.pixelFormatIndex == 1) {
            pixelFormat = V4L2_PIX_FMT_SRGGB12;
        } else {
            pixelFormat = aditof::platform::Platform::getInstance()
                              .getV4L2PixelFormat8bit();
        }

        /* Set the frame format in the driver */
        CLEAR(fmt);
        fmt.type = dev->videoBuffersType;
        fmt.fmt.pix.pixelformat = pixelFormat;
        // Raw bypass mode: frameWidthInBytes is in bytes (pixels × 2), convert to pixels
        // Standard modes: frameWidthInBytes is already in pixels
        if (type.isRawBypass) {
            fmt.fmt.pix.width = type.frameWidthInBytes / 2;
        } else {
            fmt.fmt.pix.width = type.frameWidthInBytes;
        }
        fmt.fmt.pix.height = type.frameHeightInBytes;

        if (xioctl(dev->fd, VIDIOC_S_FMT, &fmt) == -1) {
            LOG(WARNING) << "Setting Pixel Format error, errno: " << errno
                         << " error: " << strerror(errno);
            status = Status::GENERIC_ERROR;
            goto cleanup_on_error;
        }

        // For raw bypass, update stored mode details with driver-negotiated dimensions
        // Driver may reject requested dimensions and return different values
        if (type.isRawBypass && m_implData) {
            // Convert back from pixels to bytes for frameWidthInBytes
            // bytesperline is in bytes, width is in pixels
            m_implData->modeDetails.frameWidthInBytes =
                fmt.fmt.pix.bytesperline;
            m_implData->modeDetails.frameHeightInBytes = fmt.fmt.pix.height;
        }

        /* Allocate the video buffers in the driver */
        CLEAR(req);
        req.count = m_capturesPerFrame + EXTRA_BUFFERS_COUNT;
        req.type = dev->videoBuffersType;
        req.memory = V4L2_MEMORY_MMAP;

        if (xioctl(dev->fd, VIDIOC_REQBUFS, &req) == -1) {
            LOG(WARNING) << "VIDIOC_REQBUFS error "
                         << "errno: " << errno << " error: " << strerror(errno);
            status = Status::GENERIC_ERROR;
            goto cleanup_on_error;
        }

        dev->videoBuffers =
            (buffer *)calloc(req.count, sizeof(*dev->videoBuffers));
        if (!dev->videoBuffers) {
            LOG(WARNING) << "Failed to allocate video m_implData->videoBuffers";
            status = Status::GENERIC_ERROR;
            goto cleanup_on_error;
        }

        for (dev->nVideoBuffers = 0; dev->nVideoBuffers < req.count;
             dev->nVideoBuffers++) {
            CLEAR(buf);
            buf.type = dev->videoBuffersType;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = dev->nVideoBuffers;
            buf.m.planes = dev->planes;
            buf.length = 1;

            if (xioctl(dev->fd, VIDIOC_QUERYBUF, &buf) == -1) {
                LOG(WARNING)
                    << "VIDIOC_QUERYBUF error "
                    << "errno: " << errno << " error: " << strerror(errno);
                status = Status::GENERIC_ERROR;
                goto cleanup_on_error;
            }

            if (dev->videoBuffersType == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
                length = buf.length;
                offset = buf.m.offset;
            } else {
                length = buf.m.planes[0].length;
                offset = buf.m.planes[0].m.mem_offset;
            }

            dev->videoBuffers[dev->nVideoBuffers].start =
                mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED, dev->fd,
                     offset);

            if (dev->videoBuffers[dev->nVideoBuffers].start == MAP_FAILED) {
                LOG(WARNING)
                    << "mmap error "
                    << "errno: " << errno << " error: " << strerror(errno);
                status = Status::GENERIC_ERROR;
                goto cleanup_on_error;
            }

            dev->videoBuffers[dev->nVideoBuffers].length = length;
        }
    }

    if (!type.isPCM) {

        // True raw bypass = no ToFi processing (skip depth computation)
        // Lens scatter = ToFi processing required (computed from raw input)
        // m_lensScatterEnabled is set via setControl("lensScatterCompensationEnabled") from camera layer
        bool skipToFiProcessing = type.isRawBypass && !m_lensScatterEnabled;

        uint8_t bitsInAB = skipToFiProcessing ? 0 : m_bitsInAB[type.modeNumber];
        uint8_t bitsInConf =
            skipToFiProcessing ? 0 : m_bitsInConf[type.modeNumber];

        bool isADSD3100 =
            (m_implData->imagerType == SensorImagerType::IMAGER_ADSD3100);

        status = m_bufferProcessor->setVideoProperties(
            type.baseResolutionWidth, type.baseResolutionHeight,
            type.frameWidthInBytes, type.frameHeightInBytes, type.modeNumber,
            bitsInAB, bitsInConf, skipToFiProcessing, isADSD3100);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to set bufferProcessor properties!";
            goto cleanup_on_error;
        }
    }

    return status;

cleanup_on_error:
    // Clean up any partially allocated resources on error
    // Unmap buffers, close file descriptors, and reset state
    if (m_implData && m_implData->videoDevs) {
        for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
            dev = &m_implData->videoDevs[i];

            // Unmap and free video buffers
            if (dev->videoBuffers) {
                for (unsigned int j = 0; j < dev->nVideoBuffers; j++) {
                    if (dev->videoBuffers[j].start &&
                        dev->videoBuffers[j].start != MAP_FAILED) {
                        munmap(dev->videoBuffers[j].start,
                               dev->videoBuffers[j].length);
                    }
                }
                free(dev->videoBuffers);
                dev->videoBuffers = nullptr;
                dev->nVideoBuffers = 0;
            }

            // Close file descriptors
            if (dev->fd != -1) {
                if (close(dev->fd) == -1) {
                    LOG(WARNING)
                        << "Error closing fd in cleanup: "
                        << "errno: " << errno << " error: " << strerror(errno);
                }
                dev->fd = -1;
            }

            if (dev->sfd != -1) {
                if (close(dev->sfd) == -1) {
                    LOG(WARNING)
                        << "Error closing sfd in cleanup: "
                        << "errno: " << errno << " error: " << strerror(errno);
                }
                dev->sfd = -1;
            }

            // Reset state flags
            dev->started = false;
        }
    }

    // Mark sensor as closed since it's in an inconsistent state
    m_isOpen = false;

    LOG(ERROR) << "Mode change failed, sensor may need to be reopened";
    return status;
}

/**
 * @brief Retrieves a processed frame from the buffer processor.
 *
 * Gets the next available processed frame containing depth, AB, and confidence data
 * from the buffer processor's output queue. The frame has been deinterleaved and
 * processed by the ToFi compute library.
 *
 * @param[out] buffer Pointer to buffer to receive frame data
 * @param[in] index Frame index (currently unused)
 *
 * @return Status::OK on success, error status from buffer processor on failure
 */
aditof::Status Adsd3500Sensor::getFrame(uint16_t *buffer, uint32_t index) {

    using namespace aditof;
    Status status;

    if (m_depthComputeOnTarget && !m_implData->modeDetails.isPCM) {

        status = m_bufferProcessor->processBuffer(buffer);

        if (status != Status::OK) {
            LOG(ERROR) << "Failed to process buffer!";
            return status;
        }
        return status;
    }

    struct v4l2_buffer buf[MAX_SUBFRAMES_COUNT];
    struct VideoDev *dev;
    unsigned int buf_data_len;
    uint8_t *pdata;
    dev = &m_implData->videoDevs[0];
    m_capturesPerFrame = 1;
    for (int idx = 0; idx < m_capturesPerFrame; idx++) {
        status = waitForBufferPrivate(dev);
        if (status != Status::OK) {
            return status;
        }

        status = dequeueInternalBufferPrivate(buf[idx], dev);
        if (status != Status::OK) {
            return status;
        }

        status = getInternalBufferPrivate(&pdata, buf_data_len, buf[idx], dev);
        if (status != Status::OK) {
            // Re-queue buffer before returning to prevent buffer starvation
            enqueueInternalBufferPrivate(buf[idx], dev);
            return status;
        }

        memcpy(buffer, pdata, buf_data_len);

        status = enqueueInternalBufferPrivate(buf[idx], dev);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to re-queue buffer after copy";
            return status;
        }
    }

    return status;
}

aditof::Status
Adsd3500Sensor::getAvailableControls(std::vector<std::string> &controls) const {
    controls.clear();
    controls.reserve(m_controls.size());
    for (const auto &item : m_controls) {
        controls.emplace_back(item.first);
    }

    return aditof::Status::OK;
}

/**
 * @brief Sets a sensor control parameter.
 *
 * Updates the specified control parameter value and applies it via V4L2 ioctl or
 * mode selector. Supports controls like abAveraging, depthEnable, bit configurations,
 * fps, imagerType, and depth compute settings.
 *
 * @param[in] control Name of the control parameter
 * @param[in] value Value to set for the control
 *
 * @return Status::OK on success, Status::INVALID_ARGUMENT if control is unknown
 */
aditof::Status Adsd3500Sensor::setControl(const std::string &control,
                                          const std::string &value) {
    using namespace aditof;
    Status status = Status::OK;
    struct VideoDev *dev = &m_implData->videoDevs[0];

    if (m_controls.count(control) == 0) {
        LOG(WARNING) << "Unsupported control";
        return Status::INVALID_ARGUMENT;
    }

    m_controls[control] = value;

    if (control == "fps") {
        int fps = std::stoi(value);

        m_sensorFps = fps;
        status = this->adsd3500_write_cmd(0x22, m_sensorFps);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to set fps at: " << m_sensorFps
                       << "via host commands!";
            return Status::GENERIC_ERROR;
        }

        return status;
    }

    if (control == "imagerType") {
        LOG(WARNING) << "Control: " << control << " is read only!";
        return Status::UNAVAILABLE;
    }

    if (control == "depthComputeOpenSource") {
        LOG(WARNING) << "Control: " << control << " is read only!";
        return Status::UNAVAILABLE;
    }

    if (control == "availableCCBM") {
        LOG(WARNING) << "Control: " << control << " is read only!";
        return Status::UNAVAILABLE;
    }

    if (control == "disableCCBM") {
        m_controls.at("disableCCBM") = value;
        return Status::OK;
    }

    // Set the target mode for subsequent bit depth updates from runtime config.
    // Must be called before configureSensorModeDetails() so that abBits/
    // confidenceBits controls update the correct mode's arrays.
    if (control == "targetModeNumber") {
        m_targetModeNumber = (int8_t)std::stoi(value);
        LOG(INFO) << "Target mode for runtime bit config set to "
                  << (int)m_targetModeNumber;
        return Status::OK;
    }

    std::vector<std::string> convertor = {"0",  "4",  "8", "10",
                                          "12", "14", "16"};

    // Validate and convert bit depth control values
    if (control == "phaseDepthBits" || control == "abBits" ||
        control == "confidenceBits") {
        int bitIndex;
        try {
            bitIndex = std::stoi(value);
        } catch (const std::exception &e) {
            LOG(ERROR) << "Invalid bit depth value for " << control << ": "
                       << value << " (not a valid integer)";
            return Status::INVALID_ARGUMENT;
        }

        if (bitIndex < 0 || bitIndex >= static_cast<int>(convertor.size())) {
            LOG(ERROR) << "Bit depth index out of range for " << control << ": "
                       << bitIndex << " (must be 0-6)";
            return Status::INVALID_ARGUMENT;
        }

        // Determine which mode's bit arrays to update:
        // Use m_targetModeNumber if set (runtime config before setMode),
        // otherwise fall back to current mode
        uint8_t modeNum = (m_targetModeNumber >= 0)
                              ? static_cast<uint8_t>(m_targetModeNumber)
                              : m_implData->modeDetails.modeNumber;
        uint8_t actualBits = (uint8_t)std::stoi(convertor[bitIndex]);

        if (control == "phaseDepthBits") {
            m_modeSelector.setControl("depthBits", convertor[bitIndex]);
        } else if (control == "abBits") {
            m_modeSelector.setControl("abBits", convertor[bitIndex]);
            // Update runtime bit config so setMode/setVideoProperties uses correct value
            if (modeNum < m_bitsInAB.size()) {
                m_bitsInAB[modeNum] = actualBits;
                LOG(INFO) << "Runtime config: m_bitsInAB[" << (int)modeNum
                          << "] = " << (int)actualBits;
            }
        } else if (control == "confidenceBits") {
            m_modeSelector.setControl("confBits", convertor[bitIndex]);
            // Update runtime bit config so setMode/setVideoProperties uses correct value
            if (modeNum < m_bitsInConf.size()) {
                m_bitsInConf[modeNum] = actualBits;
                LOG(INFO) << "Runtime config: m_bitsInConf[" << (int)modeNum
                          << "] = " << (int)actualBits;
            }
        }
    }

    if (control == "inputFormat") {
        m_modeSelector.setControl("inputFormat", value);
        return Status::OK;
    }
    if (control == "lensScatterCompensationEnabled") {
        // Store lens scatter flag for use in setMode
        m_lensScatterEnabled = (value == "1");
        m_modeSelector.setControl("lensScatterCompensationEnabled", value);
        return Status::OK;
    }
    if (control == "enableRotation") {
        m_rotationEnabled = (value == "1");
        m_bufferProcessor->setNeedsRotation(m_rotationEnabled);
        return Status::OK;
    }
    if (control == "netlinktest") {
        return Status::OK;
    }
    // Send the command that sets the control value
    struct v4l2_control ctrl;
    memset(&ctrl, 0, sizeof(ctrl));

    ctrl.id = m_implData->controlsCommands[control];
    ctrl.value = std::stoi(value);

    if (xioctl(dev->sfd, VIDIOC_S_CTRL, &ctrl) == -1) {
        LOG(WARNING) << "Failed to set control: " << control << " "
                     << "errno: " << errno << " error: " << strerror(errno);
        status = Status::GENERIC_ERROR;
    }

    return status;
}

/**
 * @brief Retrieves a sensor control parameter value.
 *
 * Returns the current value of the specified control parameter from the internal
 * controls map.
 *
 * @param[in] control Name of the control parameter
 * @param[out] value String to receive the control value
 *
 * @return Status::OK on success, Status::INVALID_ARGUMENT if control is unknown
 */
aditof::Status Adsd3500Sensor::getControl(const std::string &control,
                                          std::string &value) const {
    using namespace aditof;

    if (m_controls.count(control) > 0) {
        if (control == "imagerType") {
            value = std::to_string((int)m_implData->imagerType);
            return Status::OK;
        }

        if (control == "depthComputeOpenSource") {
            value = m_controls.at("depthComputeOpenSource");
            return Status::OK;
        }

        if (control == "inputFormat") {
            value = m_controls.at("inputFormat");
            return Status::OK;
        }

        if (control == "disableCCBM") {
            value = m_controls.at("disableCCBM");
            return Status::OK;
        }

        if (control == "availableCCBM") {
            value = m_ccbmEnabled ? "1" : "0";
            return Status::OK;
        }

        // Send the command that reads the control value
        struct v4l2_control ctrl;
        memset(&ctrl, 0, sizeof(ctrl));

        ctrl.id = m_implData->controlsCommands[control];

        struct VideoDev *dev = &m_implData->videoDevs[0];

        if (xioctl(dev->sfd, VIDIOC_G_CTRL, &ctrl) == -1) {
            LOG(WARNING) << "Failed to get control: " << control << " "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }
        value = std::to_string(ctrl.value);

    } else {
        LOG(WARNING) << "Unsupported control";
        return Status::INVALID_ARGUMENT;
    }

    return aditof::Status::OK;
}

aditof::Status
Adsd3500Sensor::getDetails(aditof::SensorDetails &details) const {

    details = m_sensorDetails;
    return aditof::Status::OK;
}

/**
 * @brief Retrieves the sensor handle.
 *
 * Returns a pointer to the internal implementation data structure.
 *
 * @param[out] handle Pointer to receive the sensor handle
 *
 * @return Status::OK on success
 */
aditof::Status Adsd3500Sensor::getHandle(void **handle) {

    return aditof::Status::OK;
}

/**
 * @brief Retrieves the sensor name.
 *
 * Returns the name "adsd3500" identifying this sensor type.
 *
 * @param[out] name String to receive the sensor name
 *
 * @return Status::OK on success
 */
aditof::Status Adsd3500Sensor::getName(std::string &name) const {
    name = m_sensorName;

    return aditof::Status::OK;
}

/**
 * @brief Reads a command from the ADSD3500 ISP.
 *
 * Sends a read command to the ADSD3500 via the control buffer, waits for the specified
 * delay, then reads back the response data from the chip.
 *
 * @param[in] cmd Command identifier to read
 * @param[out] data Pointer to receive command response data
 * @param[in] usDelay Delay in microseconds after command execution
 *
 * @return Status::OK on success, Status::GENERIC_ERROR on ioctl failure
 */
aditof::Status Adsd3500Sensor::adsd3500_read_cmd(uint16_t cmd, uint16_t *data,
                                                 unsigned int usDelay) {
    return m_protocolManager->adsd3500_read_cmd(cmd, data, usDelay);
}

/**
 * @brief Writes a command to the ADSD3500 ISP.
 *
 * Sends a write command with data to the ADSD3500 via the control buffer, then waits
 * for the specified delay to allow the chip to process the command.
 *
 * @param[in] cmd Command identifier to write
 * @param[in] data Command data to write
 * @param[in] usDelay Delay in microseconds after command execution
 *
 * @return Status::OK on success, Status::GENERIC_ERROR on ioctl failure
 */
aditof::Status Adsd3500Sensor::adsd3500_write_cmd(uint16_t cmd, uint16_t data,
                                                  unsigned int usDelay) {
    return m_protocolManager->adsd3500_write_cmd(cmd, data, usDelay);
}

/**
 * @brief Reads payload data from ADSD3500 after sending a command.
 *
 * Sends a burst read command to the ADSD3500, waits for processing, then reads back
 * the payload data. Uses burst protocol with header, checksum, and error validation.
 *
 * @param[in] cmd Command identifier to execute before reading payload
 * @param[out] readback_data Pointer to buffer to receive payload data
 * @param[in] payload_len Length of payload data to read in bytes
 *
 * @return Status::OK on success, Status::GENERIC_ERROR on ioctl failure or checksum error
 */
aditof::Status Adsd3500Sensor::adsd3500_read_payload_cmd(uint32_t cmd,
                                                         uint8_t *readback_data,
                                                         uint16_t payload_len) {
    return m_protocolManager->adsd3500_read_payload_cmd(cmd, readback_data,
                                                        payload_len);
}

/**
 * @brief Reads payload data from ADSD3500 using burst protocol.
 *
 * Reads payload data from the ADSD3500 in chunks using the burst protocol. Validates
 * response headers, handles multi-chunk transfers, and performs checksum verification.
 *
 * @param[out] payload Pointer to buffer to receive payload data
 * @param[in] payload_len Length of payload data to read in bytes
 *
 * @return Status::OK on success, Status::GENERIC_ERROR on read error or validation failure
 */
aditof::Status Adsd3500Sensor::adsd3500_read_payload(uint8_t *payload,
                                                     uint16_t payload_len) {
    return m_protocolManager->adsd3500_read_payload(payload, payload_len);
}

aditof::Status
Adsd3500Sensor::adsd3500_write_payload_cmd(uint32_t cmd, uint8_t *payload,
                                           uint16_t payload_len) {
    return m_protocolManager->adsd3500_write_payload_cmd(cmd, payload,
                                                         payload_len);
}

/**
 * @brief Writes payload data to ADSD3500 using burst protocol.
 *
 * Writes payload data to the ADSD3500 in chunks using the burst protocol with header,
 * command, checksum, and payload. Waits for processing delay after write completion.
 *
 * @param[in] payload Pointer to payload data to write
 * @param[in] payload_len Length of payload data in bytes
 *
 * @return Status::OK on success, Status::GENERIC_ERROR on write failure
 */
aditof::Status Adsd3500Sensor::adsd3500_write_payload(uint8_t *payload,
                                                      uint16_t payload_len) {
    return m_protocolManager->adsd3500_write_payload(payload, payload_len);
}

/**
 * @brief Resets the ADSD3500 ISP chip.
 *
 * Sends a software reset command to the ADSD3500 ISP and waits for reset completion.
 *
 * @return Status::OK on success, Status::GENERIC_ERROR on reset failure
 */
aditof::Status Adsd3500Sensor::adsd3500_reset() {
    using namespace aditof;
    aditof::Status status = aditof::Status::OK;

    m_chipResetDone = false;
    m_adsd3500Status = Adsd3500Status::OK;

    // Register interrupt callback
    aditof::SensorInterruptCallback cb = [this](Adsd3500Status status) {
        m_adsd3500Status = status;
        m_chipResetDone = true;
    };
    status = adsd3500_register_interrupt_callback(cb);
    bool interruptsAvailable = (status == Status::OK);

    // Use platform-specific reset logic
    auto &platform = aditof::platform::Platform::getInstance();
    status = platform.resetSensor(interruptsAvailable, &m_chipResetDone, 10);

    if (interruptsAvailable) {
        adsd3500_unregister_interrupt_callback(cb);
    }

    return status;
}

/**
 * @brief Gets interrupt status and resets the ADSD3500 interrupt.
 *
 * Reads the interrupt status from the ADSD3500 and resets the interrupt flag.
 * Updates internal chip and imager status variables.
 *
 * @return Status::OK on success, Status::GENERIC_ERROR on read failure
 */
aditof::Status Adsd3500Sensor::adsd3500_getInterruptandReset() {
    Status status = aditof::Status::OK;

    m_chipResetDone = false;
    m_adsd3500Status = Adsd3500Status::OK;
    aditof::SensorInterruptCallback cb = [this](Adsd3500Status status) {
        m_adsd3500Status = status;
        m_chipResetDone = true;
    };

    // Register the callback
    // NOLINTNEXTLINE(clang-analyzer-optin.cplusplus.VirtualCall)
    status = Adsd3500Sensor::adsd3500_register_interrupt_callback(cb);

    // Wait for 2 sec for interrupt
    LOG(INFO) << "Waiting for interrupt.";
    int secondsTimeout = 100;
    int secondsWaited = 0;
    int secondsWaitingStep = 20;
    while (!m_chipResetDone && secondsWaited < secondsTimeout) {
        LOG(INFO) << ".";
        std::this_thread::sleep_for(
            std::chrono::milliseconds(secondsWaitingStep));
        secondsWaited += secondsWaitingStep;
    }
    LOG(INFO) << "Waited: " << secondsWaited << " ms.";
    adsd3500_unregister_interrupt_callback(cb);

    if (m_interruptAvailable != true) {
        LOG(INFO) << "Interrupt is not available , Resetting the ADSD3500";
        adsd3500_reset();
    } else {
        LOG(INFO) << "Got the Interrupt from ADSD3500";
    }

    return status;
}

/**
 * @brief Initializes depth computation on target with INI and calibration data.
 *
 * Stores INI file data and passes it along with calibration data to the buffer processor
 * for depth compute initialization. Must be called before starting frame capture.
 *
 * @param[in] iniFile Pointer to INI file data buffer
 * @param[in] iniFileLength Length of INI file in bytes
 * @param[in] calData Pointer to calibration data buffer
 * @param[in] calDataLength Length of calibration data in bytes
 *
 * @return Status::OK on success
 */
aditof::Status Adsd3500Sensor::initTargetDepthCompute(uint8_t *iniFile,
                                                      uint16_t iniFileLength,
                                                      uint8_t *calData,
                                                      uint32_t calDataLength) {
    using namespace aditof;
    Status status = Status::OK;

    // Determine ISP enable status from mode configuration instead of hardcoding
    bool ispEnabled = true; // Default to ISP enabled
    std::string modeNumberStr =
        std::to_string(m_implData->modeDetails.modeNumber);

    // Find the INI parameters for the current mode
    auto it =
        std::find_if(m_iniFileStructList.begin(), m_iniFileStructList.end(),
                     [&modeNumberStr](const iniFileStruct &iniF) {
                         return (iniF.modeName == modeNumberStr);
                     });

    if (it != m_iniFileStructList.end()) {
        // Check if depthComputeIspEnable is set in the mode configuration
        auto ispEnableIt = it->iniKeyValPairs.find("depthComputeIspEnable");
        if (ispEnableIt != it->iniKeyValPairs.end()) {
            ispEnabled = (std::stoi(ispEnableIt->second) != 0);
        } else {
            LOG(WARNING) << "depthComputeIspEnable not found for mode "
                         << modeNumberStr << ", defaulting to enabled";
        }
    } else {
        LOG(WARNING) << "INI params not found for mode " << modeNumberStr
                     << ", defaulting to ISP enabled";
    }

    status = m_bufferProcessor->setProcessorProperties(
        iniFile, iniFileLength, calData, calDataLength,
        m_implData->modeDetails.modeNumber, ispEnabled);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to initialize depth compute on target!";
        return status;
    }

    // Apply VGA frame rotation based on configuration (set via JSON or API)
    m_bufferProcessor->setNeedsRotation(m_rotationEnabled);
    LOG(INFO) << "Frame Rotation: "
              << (m_rotationEnabled ? "enabled" : "disabled");

    uint8_t depthComputeStatus;
    status = m_bufferProcessor->getDepthComputeVersion(depthComputeStatus);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to get depth compute version!";
        return status;
    }

    m_controls["depthComputeOpenSource"] = std::to_string(depthComputeStatus);

    return aditof::Status::OK;
}

/**
 * @brief Retrieves depth computation parameters.
 *
 * Returns the open-source depth compute enabled status from the buffer processor.
 *
 * @param[out] params Map to be filled with depth compute parameter key-value pairs
 *
 * @return Status::OK on success
 */
aditof::Status Adsd3500Sensor::getDepthComputeParams(
    std::map<std::string, std::string> &params) {
    TofiConfig *config = m_bufferProcessor->getTofiCongfig();
    aditof::Status status;

    ABThresholdsParams ab_params;
    int type = 3;
    status = getIniParamsImpl(&ab_params, type, config->p_tofi_cal_config);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get depth compute AB thresholds params!";
        return status;
    }
    params["abThreshMin"] = std::to_string(ab_params.ab_thresh_min);
    params["abSumThresh"] = std::to_string(ab_params.ab_sum_thresh);

    DepthRangeParams dr_params;
    type = 4;
    status = getIniParamsImpl(&dr_params, type, config->p_tofi_cal_config);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get depth compute range params!";
        return status;
    }
    params["confThresh"] = std::to_string(dr_params.conf_thresh);
    params["radialThreshMin"] = std::to_string(dr_params.radial_thresh_min);
    params["radialThreshMax"] = std::to_string(dr_params.radial_thresh_max);

    JBLFConfigParams jblf_params;
    type = 2;
    status = getIniParamsImpl(&jblf_params, type, config->p_tofi_cal_config);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get depth compute JBLF params!";
        return status;
    }
    params["jblfApplyFlag"] =
        std::to_string(static_cast<float>(jblf_params.jblf_apply_flag));
    params["jblfWindowSize"] =
        std::to_string(static_cast<float>(jblf_params.jblf_window_size));
    params["jblfGaussianSigma"] =
        std::to_string(jblf_params.jblf_gaussian_sigma);
    params["jblfExponentialTerm"] =
        std::to_string(jblf_params.jblf_exponential_term);
    params["jblfMaxEdge"] = std::to_string(jblf_params.jblf_max_edge);
    params["jblfABThreshold"] = std::to_string(jblf_params.jblf_ab_threshold);

    InputRawDataParams ir_params;
    type = 1;
    status = getIniParamsImpl(&ir_params, type, config->p_tofi_cal_config);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get depth compute input raw data params!";
        return status;
    }
    params["headerSize"] =
        std::to_string(static_cast<float>(ir_params.headerSize));

    // Add lens scatter compensation parameter
    params["lensScatterCompensationEnabled"] = m_lensScatterEnabled ? "1" : "0";

    return status;
}

/**
 * @brief Sets depth computation parameters.
 *
 * Currently a stub implementation that accepts but does not use parameters.
 *
 * @param[in] params Map of depth compute parameter key-value pairs
 *
 * @return Status::OK on success
 */
aditof::Status Adsd3500Sensor::setDepthComputeParams(
    const std::map<std::string, std::string> &params) {
    TofiConfig *config = m_bufferProcessor->getTofiCongfig();
    aditof::Status status;

    ABThresholdsParams ab_params;
    int type = 3;
    ab_params.ab_thresh_min = std::stof(params.at("abThreshMin"));
    ab_params.ab_sum_thresh = std::stof(params.at("abSumThresh"));
    status = setIniParamsImpl(&ab_params, type, config->p_tofi_cal_config);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to set depth compute AB thresholds params!";
        return status;
    }

    DepthRangeParams dr_params;
    type = 4;
    dr_params.conf_thresh = std::stof(params.at("confThresh"));
    dr_params.radial_thresh_min = std::stof(params.at("radialThreshMin"));
    dr_params.radial_thresh_max = std::stof(params.at("radialThreshMax"));
    status = setIniParamsImpl(&dr_params, type, config->p_tofi_cal_config);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to set depth compute range params!";
        return status;
    }

    JBLFConfigParams jblf_params;
    type = 2;
    status = getIniParamsImpl(
        &jblf_params, type,
        config->p_tofi_cal_config); // get any original non-customizable value
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to get depth compute JBLF params!";
        return status;
    }
    jblf_params.jblf_apply_flag =
        static_cast<int>(std::stof(params.at("jblfApplyFlag")));
    jblf_params.jblf_window_size =
        static_cast<int>(std::stof(params.at("jblfWindowSize")));
    jblf_params.jblf_gaussian_sigma = std::stof(params.at("jblfGaussianSigma"));
    jblf_params.jblf_exponential_term =
        std::stof(params.at("jblfExponentialTerm"));
    jblf_params.jblf_max_edge = std::stof(params.at("jblfMaxEdge"));
    jblf_params.jblf_ab_threshold = std::stof(params.at("jblfABThreshold"));
    status = setIniParamsImpl(&jblf_params, type, config->p_tofi_cal_config);

    // Update lens scatter compensation parameter if provided
    auto lensScatterIt = params.find("lensScatterCompensationEnabled");
    if (lensScatterIt != params.end()) {
        m_lensScatterEnabled = (lensScatterIt->second == "1");
        LOG(INFO) << "Updated lens scatter compensation: "
                  << (m_lensScatterEnabled ? "enabled" : "disabled");
    }

    return status;
}

/**
 * @brief Waits for a buffer to be available on the V4L2 device using select.
 *
 * Blocks on select() until the device is ready for reading or timeout occurs.
 *
 * @param[in] dev Pointer to VideoDev structure (uses first device if nullptr)
 *
 * @return Status::OK if buffer is ready, Status::GENERIC_ERROR on timeout or error
 */
aditof::Status Adsd3500Sensor::waitForBufferPrivate(struct VideoDev *dev) {
    return m_bufferManager->waitForBuffer(dev);
}

aditof::Status
Adsd3500Sensor::dequeueInternalBufferPrivate(struct v4l2_buffer &buf,
                                             struct VideoDev *dev) {
    return m_bufferManager->dequeueInternalBuffer(buf, dev);
}

/**
 * @brief Retrieves pointer and size of a dequeued V4L2 buffer.
 *
 * Returns a pointer to the mmap'd memory region for the given buffer index and
 * the actual number of bytes used.
 *
 * @param[out] buffer Pointer to receive buffer memory address
 * @param[out] buf_data_len Variable to receive buffer data length in bytes
 * @param[in] buf v4l2_buffer structure from dequeue operation
 * @param[in] dev Pointer to VideoDev structure (uses first device if nullptr)
 *
 * @return Status::OK on success, Status::INVALID_ARGUMENT if buffer index invalid
 */
aditof::Status Adsd3500Sensor::getInternalBufferPrivate(
    uint8_t **buffer, uint32_t &buf_data_len, const struct v4l2_buffer &buf,
    struct VideoDev *dev) {
    return m_bufferManager->getInternalBuffer(buffer, buf_data_len, buf, dev);
}

aditof::Status
Adsd3500Sensor::enqueueInternalBufferPrivate(struct v4l2_buffer &buf,
                                             struct VideoDev *dev) {
    return m_bufferManager->enqueueInternalBuffer(buf, dev);
}

/**
 * @brief Retrieves the video device file descriptor.
 *
 * Returns the file descriptor for the first video device.
 *
 * @param[out] fileDescriptor Variable to receive the file descriptor
 *
 * @return Status::OK on success, Status::GENERIC_ERROR if device not initialized
 */
aditof::Status Adsd3500Sensor::getDeviceFileDescriptor(int &fileDescriptor) {
    return m_bufferManager->getDeviceFileDescriptor(fileDescriptor);
}

/**
 * @brief Public wrapper for waitForBufferPrivate.
 *
 * Waits for a buffer to be available on the default video device.
 *
 * @return Status::OK if buffer is ready, Status::GENERIC_ERROR on error
 */
aditof::Status Adsd3500Sensor::waitForBuffer() {

    return waitForBufferPrivate();
}

/**
 * @brief Public wrapper for buffer dequeue via buffer processor.
 *
 * Dequeues a buffer from the buffer processor's internal queue.
 *
 * @param[out] buf v4l2_buffer structure to receive dequeued buffer info
 *
 * @return Status from buffer processor dequeue operation
 */
aditof::Status Adsd3500Sensor::dequeueInternalBuffer(struct v4l2_buffer &buf) {

    return dequeueInternalBufferPrivate(buf);
}

aditof::Status
Adsd3500Sensor::getInternalBuffer(uint8_t **buffer, uint32_t &buf_data_len,
                                  const struct v4l2_buffer &buf) {

    return getInternalBufferPrivate(buffer, buf_data_len, buf);
}

/**
 * @brief Public wrapper for buffer enqueue via buffer processor.
 *
 * Re-queues a buffer to the buffer processor's internal queue.
 *
 * @param[in] buf v4l2_buffer structure to re-queue
 *
 * @return Status from buffer processor enqueue operation
 */
aditof::Status Adsd3500Sensor::enqueueInternalBuffer(struct v4l2_buffer &buf) {

    return enqueueInternalBufferPrivate(buf);
}

/**
 * @brief Writes configuration block to ADSD3500 at specified offset.
 *
 * Currently a stub implementation that returns OK without action.
 *
 * @param[in] offset Offset address for configuration block
 *
 * @return Status::OK on success
 */
aditof::Status Adsd3500Sensor::writeConfigBlock(const uint32_t offset) {

    return aditof::Status::OK;
}

/**
 * @brief Queries ADSD3500 chip for firmware version and configuration.
 *
 * Reads chip ID, firmware version, imager type, reads CCB (Camera Calibration Block)
 * data including camera intrinsics and mode configurations. Initializes mode selector
 * with retrieved configuration data.
 *
 * @return Status::OK on success, Status::GENERIC_ERROR on read failure
 */
/**
 * @brief Queries the ADSD3500 chip to discover configuration.
 *
 * Delegates to ChipConfigManager to read firmware version, CCB version,
 * imager type, available modes, and INI configuration from the chip.
 *
 * @return Status::OK on success
 */
aditof::Status Adsd3500Sensor::queryAdsd3500() {
    return m_chipConfigManager->queryChipConfiguration(
        m_availableModes, m_ccbmINIContent, m_iniFileStructList, m_bitsInAB,
        m_bitsInConf, m_ccbmEnabled);
}

/**
 * @brief Registers an interrupt callback for ADSD3500 events.
 *
 * Adds the provided callback to the list of registered interrupt callbacks.
 *
 * @param[in] cb Callback function to register for interrupt events
 *
 * @return Status::OK on success
 */
aditof::Status Adsd3500Sensor::adsd3500_register_interrupt_callback(
    aditof::SensorInterruptCallback &cb) {
    if (!m_interruptManager) {
        return aditof::Status::UNAVAILABLE;
    }
    return m_interruptManager->adsd3500_register_interrupt_callback(cb);
}

/**
 * @brief Unregisters an interrupt callback for ADSD3500 events.
 *
 * Removes the provided callback from the list of registered interrupt callbacks.
 *
 * @param[in] cb Callback function to unregister from interrupt events
 *
 * @return Status::OK on success
 */
aditof::Status Adsd3500Sensor::adsd3500_unregister_interrupt_callback(
    aditof::SensorInterruptCallback &cb) {
    if (!m_interruptManager) {
        return aditof::Status::UNAVAILABLE;
    }
    return m_interruptManager->adsd3500_unregister_interrupt_callback(cb);
}

/**
 * @brief Handles ADSD3500 hardware interrupts.
 *
 * Called by the interrupt notifier when a hardware interrupt is received. Reads
 * interrupt status, updates chip/imager status, and invokes all registered callbacks.
 *
 * @param[in] signalValue Signal value from the interrupt
 *
 * @return Status::OK on success
 */
aditof::Status Adsd3500Sensor::adsd3500InterruptHandler(int signalValue) {
    return m_interruptManager->adsd3500InterruptHandler(signalValue);
}

/**
 * @brief Retrieves ADSD3500 chip and imager status.
 *
 * Returns the current chip and imager status values stored from the most recent
 * interrupt or status read.
 *
 * @param[out] chipStatus Variable to receive chip status
 * @param[out] imagerStatus Variable to receive imager status
 *
 * @return Status::OK on success
 */
aditof::Status Adsd3500Sensor::adsd3500_get_status(int &chipStatus,
                                                   int &imagerStatus) {
    using namespace aditof;
    Status status = Status::OK;

    chipStatus = m_chipStatus;
    imagerStatus = m_imagerStatus;

    return status;
}

/**
 * @brief Converts a numeric status ID to Adsd3500Status enumeration.
 *
 * Maps hardware status codes from the ADSD3500 ISP to the corresponding
 * Adsd3500Status enum values for easier error handling and reporting.
 * Handles all known status codes from the ISP firmware.
 *
 * @param[in] status Numeric status code from ADSD3500 hardware
 *
 * @return Corresponding Adsd3500Status enum value; UNKNOWN_ERROR_ID if status is unrecognized
 *
 * @note Covers status codes from 0 (OK) through 41+ (various error conditions)
 * @note Unknown status codes log an error and return UNKNOWN_ERROR_ID
 */
aditof::Adsd3500Status Adsd3500Sensor::convertIdToAdsd3500Status(int status) {
    return Adsd3500InterruptManager::convertIdToAdsd3500Status(status);
}

/**
 * @brief Retrieves INI configuration parameters.
 *
 * Gets INI parameters via TofiGetINIParams from the buffer processor's ToFi config.
 *
 * @param[out] p_config_params Pointer to configuration parameters structure
 * @param[in] params_group Parameter group identifier
 *
 * @return Status::OK on success, Status::GENERIC_ERROR if ToFi config unavailable
 */
aditof::Status Adsd3500Sensor::getIniParamsImpl(void *p_config_params,
                                                int params_group,
                                                const void *p_tofi_cal_config) {
    return m_iniConfigManager->getIniParamsImpl(p_config_params, params_group,
                                                p_tofi_cal_config);
}

/**
 * @brief Sets INI configuration parameters.
 *
 * Sets INI parameters via TofiSetINIParams to the buffer processor's ToFi config.
 *
 * @param[in] p_config_params Pointer to configuration parameters structure
 * @param[in] params_group Parameter group identifier
 *
 * @return Status::OK on success, Status::GENERIC_ERROR if ToFi config unavailable
 */
aditof::Status Adsd3500Sensor::setIniParamsImpl(void *p_config_params,
                                                int params_group,
                                                const void *p_tofi_cal_config) {
    return m_iniConfigManager->setIniParamsImpl(p_config_params, params_group,
                                                p_tofi_cal_config);
}

/**
 * @brief Retrieves default INI parameters for a specific mode.
 *
 * Reads the default INI file for the specified mode from the modes directory and
 * converts it to an INI structure.
 *
 * @param[in] mode Mode number to retrieve default parameters for
 * @param[out] iniParams INI structure to populate with default parameters
 *
 * @return Status::OK on success, Status::GENERIC_ERROR if file read or parse fails
 */
aditof::Status Adsd3500Sensor::getDefaultIniParamsForMode(
    const std::string &imager, const std::string &mode,
    std::map<std::string, std::string> &params) {
    return m_iniConfigManager->getDefaultIniParamsForMode(imager, mode, params);
}

aditof::Status
Adsd3500Sensor::mergeIniParams(std::vector<iniFileStruct> &iniFileStructList) {
    return m_iniConfigManager->mergeIniParams(iniFileStructList);
}

/**
 * @brief Converts INI parameters between structure and string formats.
 *
 * Converts an INI file structure to a string representation.
 *
 * @param[in] iniStruct INI structure containing parameters
 * @param[out] iniString String to receive INI content
 *
 * @return Status::OK on success
 */
aditof::Status Adsd3500Sensor::convertIniParams(iniFileStruct &iniStruct,
                                                std::string &inistr) {
    return m_iniConfigManager->convertIniParams(iniStruct, inistr);
}

/**
 * @brief Retrieves INI parameters as a string array for a specific mode.
 *
 * Gets the default INI parameters for the mode and converts them to string format.
 *
 * @param[in] mode Mode number to retrieve INI parameters for
 * @param[out] iniStr String to receive INI parameters
 *
 * @return Status::OK on success, Status::GENERIC_ERROR on failure
 */
aditof::Status Adsd3500Sensor::getIniParamsArrayForMode(int mode,
                                                        std::string &iniStr) {
    return m_iniConfigManager->getIniParamsArrayForMode(mode, iniStr);
}
#pragma region Stream_Recording_and_Playback

/**
 * @brief Starts recording frames to a file.
 *
 * Delegates to the buffer processor's startRecording method to begin recording
 * processed frames to a binary file.
 *
 * @param[out] fileName String to receive the generated recording file path
 * @param[in] parameters Pointer to header parameters buffer
 * @param[in] paramSize Size of header parameters in bytes
 *
 * @return Status from buffer processor startRecording
 */
aditof::Status Adsd3500Sensor::startRecording(std::string &fileName,
                                              uint8_t *parameters,
                                              uint32_t paramSize) {

    using namespace aditof;
    LOG(INFO) << __func__ << ": Start recording";
    m_bufferProcessor->startRecording(fileName, (uint8_t *)parameters,
                                      paramSize);
    Status status = Status::OK;

    return status;
}

/**
 * @brief Stops recording frames.
 *
 * Delegates to the buffer processor's stopRecording method to finalize and close
 * the recording file.
 *
 * @return Status from buffer processor stopRecording
 */
aditof::Status Adsd3500Sensor::stopRecording() {

    using namespace aditof;

    Status status = Status::OK;

    status = m_bufferProcessor->stopRecording();

    return status;
}

/**
 * @brief Sets the playback file for offline replay.
 *
 * Currently returns unavailable as playback is not supported on target sensors.
 *
 * @param[in] filePath Path to the playback file
 *
 * @return Status::UNAVAILABLE (playback not supported on target)
 */
aditof::Status Adsd3500Sensor::setPlaybackFile(const std::string filePath) {

    return aditof::Status::GENERIC_ERROR;
}

/**
 * @brief Stops playback.
 *
 * Currently returns unavailable as playback is not supported on target sensors.
 *
 * @return Status::UNAVAILABLE (playback not supported on target)
 */
aditof::Status Adsd3500Sensor::stopPlayback() {

    return aditof::Status::GENERIC_ERROR;
}

/**
 * @brief Retrieves the frame count from playback file.
 *
 * Currently returns unavailable as playback is not supported on target sensors.
 *
 * @param[out] frameCount Variable to receive frame count
 *
 * @return Status::UNAVAILABLE (playback not supported on target)
 */
aditof::Status Adsd3500Sensor::getFrameCount(uint32_t &frameCount) {
    frameCount = m_frameIndex.size();

    return aditof::Status::OK;
}

/**
 * @brief Retrieves the header from playback file.
 *
 * Currently returns unavailable as playback is not supported on target sensors.
 *
 * @param[out] buffer Pointer to buffer to receive header data
 * @param[in] bufferSize Size of the buffer in bytes
 *
 * @return Status::UNAVAILABLE (playback not supported on target)
 */
aditof::Status Adsd3500Sensor::getHeader(uint8_t *buffer, uint32_t bufferSize) {

    return aditof::Status::GENERIC_ERROR;
}

/**
 * @brief Reads a frame from playback file.
 *
 * Currently returns unavailable as playback is not supported on target sensors.
 *
 * @param[out] buffer Pointer to buffer to receive frame data
 * @param[in,out] bufferSize On input: buffer size; on output: actual size read
 * @param[in] index Frame index to read
 *
 * @return Status::UNAVAILABLE (playback not supported on target)
 */
aditof::Status Adsd3500Sensor::readFrame(uint8_t *buffer, uint32_t &bufferSize,
                                         uint32_t index) {

    return aditof::Status::GENERIC_ERROR;
}

#pragma endregion
