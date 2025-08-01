/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#include "adsd3500_sensor.h"
#include "aditof/frame_operations.h"
#include "adsd3500_interrupt_notifier.h"
#include "gpio.h"
#include "sensor-tables/device_parameters.h"
#include "utils.h"
#include "utils_ini.h"

#include <algorithm>
#include <arm_neon.h>
#include <cmath>
#include <fcntl.h>
#include <fstream>
#ifdef USE_GLOG
#include <glog/logging.h>
#else
#include <aditof/log.h>
#include <cstring>
#include <unistd.h>
#endif
#include "tofi/tofi_config.h"
#include <dirent.h>
#include <linux/videodev2.h>
#include <signal.h>
#include <sstream>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <thread>
#include <unordered_map>

#define MAX_SUBFRAMES_COUNT                                                    \
    10 // maximum number of subframes that are used to create a full frame (maximum total_captures of all modes)
#define EXTRA_BUFFERS_COUNT                                                    \
    3 // how many extra buffers are sent to the driver in addition to the total_captures of a mode

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define V4L2_CID_AD_DEV_CHIP_CONFIG (0x9819e1)
#define CTRL_PACKET_SIZE 65537
#define CTRL_SET_MODE (0x9819e0)
#define CTRL_AB_AVG (0x9819e5)
#define CTRL_DEPTH_EN (0x9819e6)
#define CTRL_PHASE_DEPTH_BITS (0x9819e2)
#define CTRL_AB_BITS (0x9819e3)
#define CTRL_CONFIDENCE_BITS (0x9819e4)
#ifdef NVIDIA
#define CTRL_SET_FRAME_RATE (0x9a200b)
#endif
#define ADSD3500_CTRL_PACKET_SIZE 4099
// Can be moved to target_definitions in "camera"/"platform"
#define TEMP_SENSOR_DEV_PATH "/dev/i2c-1"
#define LASER_TEMP_SENSOR_I2C_ADDR 0x49
#define AFE_TEMP_SENSOR_I2C_ADDR 0x4b

#define ADI_DEBUG 1
#define REQ_COUNT 10

#define NR_OF_MODES_FROM_CCB 10
#define SIZE_OF_MODES_FROM_CCB 256

uint16_t chip_id;
uint8_t mode_num;

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

enum class SensorImagerType {
    IMAGER_UNKNOWN,
    IMAGER_ADSD3100,
    IMAGER_ADSD3030,
    IMAGER_ADTF3080
};

enum class CCBVersion { CCB_UNKNOWN, CCB_VERSION0, CCB_VERSION1, CCB_VERSION2 };

struct Adsd3500Sensor::ImplData {
    uint8_t numVideoDevs;
    struct VideoDev *videoDevs;
    aditof::DepthSensorModeDetails modeDetails;
    std::unordered_map<std::string, __u32> controlsCommands;
    SensorImagerType imagerType;
    CCBVersion ccbVersion;
    std::string fw_ver;

    ImplData()
        : numVideoDevs(1),
          videoDevs(nullptr), imagerType{SensorImagerType::IMAGER_UNKNOWN},
          ccbVersion{CCBVersion::CCB_UNKNOWN}, modeDetails{0, {}, 0, 0, 0, 0,
                                                           0, 0,  0, 0, {}} {}
};

// TO DO: This exists in linux_utils.h which is not included on Dragoboard.
// Should not have duplicated code if possible.
static int xioctl(int fh, unsigned int request, void *arg) {
    int r;
    int tries = 3;

    do {
        r = ioctl(fh, request, arg);
    } while (--tries > 0 && r == -1 && EINTR == errno);

    return r;
}

Adsd3500Sensor::Adsd3500Sensor(const std::string &driverPath,
                               const std::string &driverSubPath,
                               const std::string &captureDev)
    : m_driverPath(driverPath), m_driverSubPath(driverSubPath),
      m_captureDev(captureDev), m_implData(new Adsd3500Sensor::ImplData),
      m_firstRun(true), m_adsd3500Queried(false), m_depthComputeOnTarget(true),
      m_chipStatus(0), m_imagerStatus(0),
      m_hostConnectionType(aditof::ConnectionType::ON_TARGET) {
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

    // Define the commands that correspond to the sensor controls
    m_implData->controlsCommands["abAveraging"] = 0x9819e5;
    m_implData->controlsCommands["depthEnable"] = 0x9819e6;
    m_implData->controlsCommands["phaseDepthBits"] = 0x9819e2;
    m_implData->controlsCommands["abBits"] = 0x9819e3;
    m_implData->controlsCommands["confidenceBits"] = 0x9819e4;

    m_bufferProcessor = new BufferProcessor();
}

Adsd3500Sensor::~Adsd3500Sensor() {
    struct VideoDev *dev;

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

    delete m_bufferProcessor;
}

aditof::Status Adsd3500Sensor::open() {
    using namespace aditof;
    Status status = Status::OK;

    // Subscribe to ADSD3500 interrupts
    if (Adsd3500InterruptNotifier::getInstance().interruptsAvailable()) {
        std::weak_ptr<Adsd3500Sensor> wptr = shared_from_this();
        Adsd3500InterruptNotifier::getInstance().subscribeSensor(wptr);
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
    for (const auto card : cards) {
        LOG(INFO) << card;
    }

    m_implData->numVideoDevs = driverSubPaths.size();
    m_implData->videoDevs = new VideoDev[m_implData->numVideoDevs];

    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        devName = driverPaths.at(i).c_str();
        subDevName = driverSubPaths.at(i).c_str();
        cardName = cards.at(i).c_str();
        dev = &m_implData->videoDevs[i];

        LOG(INFO) << "device: " << devName << "\tsubdevice: " << subDevName;

        //Don't open the video device for UVC context. It is opened in uvc-app/lib/v4l2.c
        if (m_hostConnectionType != ConnectionType::USB) {
            /* Open V4L2 device */
            if (stat(devName, &st) == -1) {
                LOG(WARNING)
                    << "Cannot identify " << devName << "errno: " << errno
                    << "error: " << strerror(errno);
                return Status::GENERIC_ERROR;
            }

            if (!S_ISCHR(st.st_mode)) {
                LOG(WARNING) << devName << " is not a valid device";
                return Status::GENERIC_ERROR;
            }

            dev->fd = ::open(devName, O_RDWR | O_NONBLOCK, 0);
            if (dev->fd == -1) {
                LOG(WARNING) << "Cannot open " << devName << "errno: " << errno
                             << "error: " << strerror(errno);
                return Status::GENERIC_ERROR;
            }

            if (xioctl(dev->fd, VIDIOC_QUERYCAP, &cap) == -1) {
                LOG(WARNING) << devName << " VIDIOC_QUERYCAP error";
                return Status::GENERIC_ERROR;
            }

            if (strncmp((char *)cap.card, cardName, strlen(cardName))) {
                LOG(WARNING) << "CAPTURE Device " << cap.card;
                LOG(WARNING) << "Read " << cardName;
                return Status::GENERIC_ERROR;
            }

            if (!(cap.capabilities &
                  (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE))) {
                LOG(WARNING) << devName << " is not a video capture device";
                return Status::GENERIC_ERROR;
            }

            if (cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE) {
                dev->videoBuffersType = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            } else {
                dev->videoBuffersType = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            }

            if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
                LOG(WARNING) << devName << " does not support streaming i/o";
                return Status::GENERIC_ERROR;
            }
        }
        /* Open V4L2 subdevice */
        if (stat(subDevName, &st) == -1) {
            LOG(WARNING) << "Cannot identify " << subDevName
                         << " errno: " << errno
                         << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

        if (!S_ISCHR(st.st_mode)) {
            LOG(WARNING) << subDevName << " is not a valid device";
            return Status::GENERIC_ERROR;
        }

        dev->sfd = ::open(subDevName, O_RDWR | O_NONBLOCK);
        if (dev->sfd == -1) {
            LOG(WARNING) << "Cannot open " << subDevName << " errno: " << errno
                         << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
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

            for (int i = 0; i < 10; i++) {
                // adsd3500_reset();

                adsd3500StateStatus = adsd3500_read_cmd(0x0020, &adsd3500State);
                if (adsd3500StateStatus == Status::OK) {
                    if ((adsd3500State != 0x0) && (adsd3500State != 0x29)) {
                        LOG(INFO) << "ADSD3500 is not in good state, Resetting "
                                     "the ADSD3500";
                        adsd3500_reset();
                    }
                } else {
                    LOG(INFO) << "Could not read the status register, "
                                 "Resetting ADSD3500";
                    adsd3500_reset();
                }
#ifdef DUAL
                chipIDStatus = adsd3500_read_cmd(0x0116, &chipID, 110 * 1000);
#else
                chipIDStatus = adsd3500_read_cmd(0x0112, &chipID);
#endif
                if (chipIDStatus != Status::OK) {
                    LOG(INFO) << "Could not read chip ID. Resetting ADSD3500 "
                                 "to handle previous error.";
                    adsd3500_reset();
                }

                chip_id = chipID;

                dealiasStatus =
                    adsd3500_read_payload_cmd(0x02, dealiasCheck, 32);
                if (dealiasStatus != Status::OK) {
                    LOG(INFO) << "Could not read Dealias Parameters. Resetting "
                                 "ADSD3500 "
                                 "to handle previous error.";
                    adsd3500_reset();
                    continue;
                }

                if (chipIDStatus == aditof::Status::OK &&
                    dealiasStatus == aditof::Status::OK) {
                    LOG(INFO) << "ADSD3500 is ready to communicate with.";
                    break;
                }
            }

            if (chipIDStatus != aditof::Status::OK) {
                LOG(ERROR) << "Cannot read chip id! Latest ADSD3500 "
                              "programming might not be succesful";
                return chipIDStatus;
            }

            if (dealiasStatus != aditof::Status::OK) {
                LOG(ERROR) << "Cannot read dealias parameters! Latest ADSD3500 "
                              "programming might not be succesful";
                return dealiasStatus;
            }

            m_firstRun = false;
        }
    }

    if (!m_adsd3500Queried) {
        status = queryAdsd3500();
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

    return status;
}

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

aditof::Status Adsd3500Sensor::stop() {
    using namespace aditof;
    Status status = Status::OK;
    struct VideoDev *dev;

    m_bufferProcessor->stopThreads();
    for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
        dev = &m_implData->videoDevs[i];

        if (!dev->started) {
            LOG(INFO) << "Device " << i << " already stopped";
            return Status::OK;
        }
        LOG(INFO) << "Stopping device";

        if (xioctl(dev->fd, VIDIOC_STREAMOFF, &dev->videoBuffersType) != 0) {
            LOG(WARNING) << "VIDIOC_STREAMOFF error "
                         << "errno: " << errno << " error: " << strerror(errno);
            return Status::GENERIC_ERROR;
        }

        dev->started = false;
    }
    status = adsd3500_getInterruptandReset();
    return status;
}

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

    status = setMode(modeTable);
    if (status != aditof::Status::OK) {
        LOG(ERROR) << "Failed to set mode for the current configuration!";
        return aditof::Status::GENERIC_ERROR;
    }

    m_implData->modeDetails = modeTable;
    return aditof::Status::OK;
}

aditof::Status
Adsd3500Sensor::setMode(const aditof::DepthSensorModeDetails &type) {

    using namespace aditof;
    Status status = Status::OK;
    struct VideoDev *dev;

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

    status = open();
    if (status != aditof::Status::OK) {
        LOG(INFO) << "Failed to open sensor!";
        return status;
    }

    // Don't request buffers & set fromat for UVC context. It is already done in uvc-app/lib/v4l2.c
    if (m_hostConnectionType != ConnectionType::USB) {
        m_capturesPerFrame = 1;

        for (unsigned int i = 0; i < m_implData->numVideoDevs; i++) {
            dev = &m_implData->videoDevs[i];

            //Set mode in chip code block
            struct v4l2_requestbuffers req;
            struct v4l2_buffer buf;
            struct v4l2_format fmt;
            size_t length, offset;

            static struct v4l2_control ctrl;

            memset(&ctrl, 0, sizeof(ctrl));

            ctrl.id = CTRL_SET_MODE;
            ctrl.value = type.modeNumber;
            mode_num = type.modeNumber;

            if (xioctl(dev->sfd, VIDIOC_S_CTRL, &ctrl) == -1) {
                LOG(WARNING)
                    << "Setting Mode error "
                    << "errno: " << errno << " error: " << strerror(errno);
                status = Status::GENERIC_ERROR;
                return status;
            }

            //End of set mode in chip

            if (type.modeNumber != m_implData->modeDetails.modeNumber) {
                for (unsigned int i = 0; i < dev->nVideoBuffers; i++) {
                    if (munmap(dev->videoBuffers[i].start,
                               dev->videoBuffers[i].length) == -1) {
                        LOG(WARNING) << "munmap error "
                                     << "errno: " << errno
                                     << " error: " << strerror(errno);
                        return Status::GENERIC_ERROR;
                    }
                }
                free(dev->videoBuffers);
                dev->nVideoBuffers = 0;
                CLEAR(req);
                req.count = 0;
                req.type = dev->videoBuffersType;
                req.memory = V4L2_MEMORY_MMAP;

                if (xioctl(dev->fd, VIDIOC_REQBUFS, &req) == -1) {
                    LOG(WARNING)
                        << "VIDIOC_REQBUFS error "
                        << "errno: " << errno << " error: " << strerror(errno);
                    return Status::GENERIC_ERROR;
                }
            } else if (dev->nVideoBuffers) {
                return status;
            }

            __u32 pixelFormat = 0;

            if (type.pixelFormatIndex == 1) {
                pixelFormat = V4L2_PIX_FMT_SBGGR12;
            } else {
#ifdef NXP
                pixelFormat = V4L2_PIX_FMT_SBGGR8;
#else
                pixelFormat = V4L2_PIX_FMT_SRGGB8;
#endif
            }

            /* Set the frame format in the driver */
            CLEAR(fmt);
            fmt.type = dev->videoBuffersType;
            fmt.fmt.pix.pixelformat = pixelFormat;
            fmt.fmt.pix.width = type.frameWidthInBytes;
            fmt.fmt.pix.height = type.frameHeightInBytes;
            if (xioctl(dev->fd, VIDIOC_S_FMT, &fmt) == -1) {
                LOG(WARNING) << "Setting Pixel Format error, errno: " << errno
                             << " error: " << strerror(errno);
                return Status::GENERIC_ERROR;
            }

            /* Allocate the video buffers in the driver */
            CLEAR(req);
            req.count = m_capturesPerFrame + EXTRA_BUFFERS_COUNT;
            req.type = dev->videoBuffersType;
            req.memory = V4L2_MEMORY_MMAP;

            if (xioctl(dev->fd, VIDIOC_REQBUFS, &req) == -1) {
                LOG(WARNING)
                    << "VIDIOC_REQBUFS error "
                    << "errno: " << errno << " error: " << strerror(errno);
                return Status::GENERIC_ERROR;
            }

            dev->videoBuffers =
                (buffer *)calloc(req.count, sizeof(*dev->videoBuffers));
            if (!dev->videoBuffers) {
                LOG(WARNING)
                    << "Failed to allocate video m_implData->videoBuffers";
                return Status::GENERIC_ERROR;
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
                    return Status::GENERIC_ERROR;
                }

                if (dev->videoBuffersType == V4L2_BUF_TYPE_VIDEO_CAPTURE) {
                    length = buf.length;
                    offset = buf.m.offset;
                } else {
                    length = buf.m.planes[0].length;
                    offset = buf.m.planes[0].m.mem_offset;
                }

                dev->videoBuffers[dev->nVideoBuffers].start =
                    mmap(NULL, length, PROT_READ | PROT_WRITE, MAP_SHARED,
                         dev->fd, offset);

                if (dev->videoBuffers[dev->nVideoBuffers].start == MAP_FAILED) {
                    LOG(WARNING)
                        << "mmap error "
                        << "errno: " << errno << " error: " << strerror(errno);
                    return Status::GENERIC_ERROR;
                }

                dev->videoBuffers[dev->nVideoBuffers].length = length;
            }
        }
    }

    if (!type.isPCM) {
        //TO DO: update this values when frame_impl gets restructured
        status = m_bufferProcessor->setVideoProperties(
            type.baseResolutionWidth, type.baseResolutionHeight,
            type.frameWidthInBytes, type.frameHeightInBytes, type.modeNumber);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to set bufferProcessor properties!";
            return status;
        }
    }

    return status;
}

aditof::Status Adsd3500Sensor::getFrame(uint16_t *buffer) {

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
            return status;
        }

        memcpy(buffer, pdata, buf_data_len);

        status = enqueueInternalBufferPrivate(buf[idx], dev);
        if (status != Status::OK) {
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
#ifdef NVIDIA
        struct v4l2_ext_control extCtrl;
        struct v4l2_ext_controls extCtrls;
        memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
        memset(&extCtrl, 0, sizeof(struct v4l2_ext_control));

        extCtrls.count = 1;
        extCtrls.controls = &extCtrl;
        extCtrl.id = CTRL_SET_FRAME_RATE;
        extCtrl.value = fps;

        if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
            LOG(WARNING) << "Failed to set control:  " << control << " "
                         << "errno: " << errno << " error: " << strerror(errno);
            status = Status::GENERIC_ERROR;
        }
#else // NXP
        struct v4l2_streamparm fpsControl;
        memset(&fpsControl, 0, sizeof(struct v4l2_streamparm));

        fpsControl.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fpsControl.parm.capture.timeperframe.numerator = 1;
        fpsControl.parm.capture.timeperframe.denominator = fps;

        if (xioctl(dev->fd, VIDIOC_S_PARM, &fpsControl) == -1) {
            LOG(WARNING) << "Failed to set control: " << control << " "
                         << "errno: " << errno << " error: " << strerror(errno);
            status = Status::GENERIC_ERROR;
        }
#endif

        m_sensorFps = fps;
        status = this->adsd3500_write_cmd(0x22, fps);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to set fps at: " << fps
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

    std::vector<std::string> convertor = {"0",  "4",  "8", "10",
                                          "12", "14", "16"};

    if (control == "phaseDepthBits")
        m_modeSelector.setControl("depthBits", convertor.at(stoi(value)));
    if (control == "abBits")
        m_modeSelector.setControl("abBits", convertor.at(stoi(value)));
    if (control == "confidenceBits")
        m_modeSelector.setControl("confBits", convertor.at(stoi(value)));
    if (control == "inputFormat") {
        m_modeSelector.setControl("inputFormat", value);
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

aditof::Status Adsd3500Sensor::getHandle(void **handle) {

    return aditof::Status::OK;
}

aditof::Status Adsd3500Sensor::getName(std::string &name) const {
    name = m_sensorName;

    return aditof::Status::OK;
}

aditof::Status
Adsd3500Sensor::setHostConnectionType(std::string &connectionType) {
    if (connectionType == "USB") {
        m_hostConnectionType = aditof::ConnectionType::USB;
    } else if (connectionType == "NETWORK") {
        m_hostConnectionType = aditof::ConnectionType::NETWORK;
    }

    return aditof::Status::OK;
}

aditof::Status Adsd3500Sensor::adsd3500_read_cmd(uint16_t cmd, uint16_t *data,
                                                 unsigned int usDelay) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 1;
    buf[1] = 0;
    buf[2] = 2;
    buf[3] = uint8_t(cmd >> 8);
    buf[4] = uint8_t(cmd & 0xFF);
    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    buf[0] = 0;
    buf[1] = 0;
    buf[2] = 2;

    extCtrl.p_u8 = buf;

    usleep(usDelay);

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    if (xioctl(dev->sfd, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not get control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    *data = (uint16_t)(extCtrl.p_u8[3] << 8) + (uint16_t)(extCtrl.p_u8[4]);

    return status;
}

aditof::Status Adsd3500Sensor::adsd3500_write_cmd(uint16_t cmd, uint16_t data,
                                                  unsigned int usDelay) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 1;
    buf[1] = 0;
    buf[2] = 4;
    buf[3] = uint8_t(cmd >> 8);
    buf[4] = uint8_t(cmd & 0xFF);
    buf[5] = uint8_t(data >> 8);
    buf[6] = uint8_t(data & 0xFF);
    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    if (usDelay)
        usleep(usDelay);

    return status;
}

// TO DO: Verify mechanism for read/write burst

aditof::Status Adsd3500Sensor::adsd3500_read_payload_cmd(uint32_t cmd,
                                                         uint8_t *readback_data,
                                                         uint16_t payload_len) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    //switch to burst mode
    uint32_t switchCmd = 0x0019;
    uint16_t switchPayload = 0x0000;

    status = adsd3500_write_cmd(switchCmd, switchPayload);
    if (status != Status::OK) {
        LOG(INFO) << "Failed to switch to burst mode!";
        return status;
    }

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];
    memset(buf, 0, ADSD3500_CTRL_PACKET_SIZE * sizeof(uint8_t));

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;

    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 0x01;
    buf[1] = 0x00;
    buf[2] = 0x10;

    buf[3] = 0xAD;
    buf[6] = uint8_t(cmd & 0xFF);

    uint32_t checksum = 0;
    for (int i = 0; i < 7; i++) {
        checksum += buf[i + 4];
    }
    memcpy(buf + 11, &checksum, 4);
    memcpy(buf + 15, readback_data, 1);
    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    if (cmd == 0x13)
        usleep(1000);
    else if (cmd == 0x19)
        usleep(5000);

    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 0x00;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);

    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    if (xioctl(dev->sfd, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not get control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    memcpy(readback_data, extCtrl.p_u8 + 3, payload_len);

    //If we use the read ccb command we need to keep adsd3500 in burst mode
    if (cmd == 0x13) {
        return status;
    }

    //switch to standard mode
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    uint8_t switchBuf[] = {0x01, 0x00, 0x10, 0xAD, 0x00, 0x00, 0x10,
                           0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00};

    memcpy(extCtrl.p_u8, switchBuf, 19);

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << " (switch to standard mode)"
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status Adsd3500Sensor::adsd3500_read_payload(uint8_t *payload,
                                                     uint16_t payload_len) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];
    memset(buf, 0, ADSD3500_CTRL_PACKET_SIZE * sizeof(uint8_t));

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;

    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 0x00;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);

    extCtrl.p_u8 = buf;

    usleep(30000);

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " to read payload with length: " << payload_len
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    if (xioctl(dev->sfd, VIDIOC_G_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not get control: 0x" << std::hex << extCtrl.id
                     << " to read payload with length: " << payload_len
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    memcpy(payload, extCtrl.p_u8 + 3, payload_len);

    return status;
}

aditof::Status
Adsd3500Sensor::adsd3500_write_payload_cmd(uint32_t cmd, uint8_t *payload,
                                           uint16_t payload_len) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    //switch to burst mode
    uint32_t switchCmd = 0x0019;
    uint16_t switchPayload = 0x0000;

    status = adsd3500_write_cmd(switchCmd, switchPayload);
    if (status != Status::OK) {
        LOG(INFO) << "Failed to switch to burst mode!";
    }

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    payload_len += 16;
    buf[0] = 0x01;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);

    payload_len -= 16;
    buf[3] = 0xAD;
    buf[4] = uint8_t(payload_len >> 8);
    buf[5] = uint8_t(payload_len & 0xFF);
    buf[6] = uint8_t(cmd & 0xFF);

    uint32_t checksum = 0;
    for (int i = 0; i < 7; i++) {
        checksum += buf[i + 4];
    }
    memcpy(buf + 11, &checksum, 4);
    memcpy(buf + 15, payload, payload_len);
    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    //switch to standard mode
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    uint8_t switchBuf[] = {0x01, 0x00, 0x10, 0xAD, 0x00, 0x00, 0x10,
                           0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00,
                           0x00, 0x00, 0x00, 0x00, 0x00};

    memcpy(extCtrl.p_u8, switchBuf, 19);

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " with command: 0x" << std::hex << cmd
                     << " (switch to standard mode)"
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status Adsd3500Sensor::adsd3500_write_payload(uint8_t *payload,
                                                      uint16_t payload_len) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];
    Status status = Status::OK;

    static struct v4l2_ext_control extCtrl;
    static struct v4l2_ext_controls extCtrls;
    static uint8_t buf[ADSD3500_CTRL_PACKET_SIZE];

    extCtrl.size = ADSD3500_CTRL_PACKET_SIZE;
    extCtrl.id = V4L2_CID_AD_DEV_CHIP_CONFIG;
    memset(&extCtrls, 0, sizeof(struct v4l2_ext_controls));
    extCtrls.controls = &extCtrl;
    extCtrls.count = 1;

    buf[0] = 1;
    buf[1] = uint8_t(payload_len >> 8);
    buf[2] = uint8_t(payload_len & 0xFF);

    memcpy(buf + 3, payload, payload_len);
    extCtrl.p_u8 = buf;

    if (xioctl(dev->sfd, VIDIOC_S_EXT_CTRLS, &extCtrls) == -1) {
        LOG(WARNING) << "Could not set control: 0x" << std::hex << extCtrl.id
                     << " to write payload with length: " << payload_len
                     << ". Reason: " << strerror(errno) << "(" << errno << ")";
        return Status::GENERIC_ERROR;
    }

    usleep(100000);

    return status;
}

aditof::Status Adsd3500Sensor::adsd3500_reset() {
    using namespace aditof;
    Status status = Status::OK;

#if defined(NXP)
    m_chipResetDone = false;
    m_adsd3500Status = Adsd3500Status::OK;
    aditof::SensorInterruptCallback cb = [this](Adsd3500Status status) {
        m_adsd3500Status = status;
        m_chipResetDone = true;
    };
    status = adsd3500_register_interrupt_callback(cb);
    bool interruptsAvailable = (status == Status::OK);

#ifdef DUAL
    system("echo 0 > /sys/class/gpio/gpio64/value");
    usleep(1000000);
    system("echo 1 > /sys/class/gpio/gpio64/value");
#else
    system("echo 0 > /sys/class/gpio/gpio122/value");
    usleep(1000000);
    system("echo 1 > /sys/class/gpio/gpio122/value");
#endif

    if (interruptsAvailable) {
        LOG(INFO) << "Waiting for ADSD3500 to reset.";
        int secondsTimeout = 10;
        int secondsWaited = 0;
        int secondsWaitingStep = 1;
        while (!m_chipResetDone && secondsWaited < secondsTimeout) {
            LOG(INFO) << ".";
            std::this_thread::sleep_for(
                std::chrono::seconds(secondsWaitingStep));
            secondsWaited += secondsWaitingStep;
        }
        LOG(INFO) << "Waited: " << secondsWaited << " seconds";
        adsd3500_unregister_interrupt_callback(cb);
    } else {
        usleep(7000000);
    }
#elif defined(NVIDIA)
    struct stat st;
    if (stat("/sys/class/gpio/PP.04/value", &st) == 0) {
        system("echo 0 > /sys/class/gpio/PP.04/value");
        usleep(100000);
        system("echo 1 > /sys/class/gpio/PP.04/value");
        usleep(5000000);
    } else {
        Gpio gpio11("/dev/gpiochip3", 11);
        gpio11.openForWrite();

        gpio11.writeValue(0);
        usleep(100000);
        gpio11.writeValue(1);
        usleep(5000000);

        gpio11.close();
    }
#endif
    return aditof::Status::OK;
}

aditof::Status Adsd3500Sensor::adsd3500_getInterruptandReset() {
    Status status = aditof::Status::OK;
#if defined(NXP)
    m_chipResetDone = false;
    m_adsd3500Status = Adsd3500Status::OK;
    aditof::SensorInterruptCallback cb = [this](Adsd3500Status status) {
        m_adsd3500Status = status;
        m_chipResetDone = true;
    };

    // Register the callback
    status = adsd3500_register_interrupt_callback(cb);

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

#endif

    return status;
}

aditof::Status Adsd3500Sensor::initTargetDepthCompute(uint8_t *iniFile,
                                                      uint16_t iniFileLength,
                                                      uint8_t *calData,
                                                      uint16_t calDataLength) {
    using namespace aditof;
    Status status = Status::OK;

    status = m_bufferProcessor->setProcessorProperties(
        iniFile, iniFileLength, calData, calDataLength,
        m_implData->modeDetails.modeNumber, true);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to initialize depth compute on target!";
        return status;
    }

    uint8_t depthComputeStatus;
    status = m_bufferProcessor->getDepthComputeVersion(depthComputeStatus);
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to get depth compute version!";
        return status;
    }

    m_controls["depthComputeOpenSource"] = std::to_string(depthComputeStatus);

    return aditof::Status::OK;
}

aditof::Status Adsd3500Sensor::getDepthComputeParams(
    std::map<std::string, std::string> &params) {
    TofiConfig *config = m_bufferProcessor->getTofiCongfig();
    aditof::Status status;

    ABThresholdsParams ab_params;
    int type = 3;
    status = getIniParamsImpl(&ab_params, type, config->p_tofi_cal_config);
    params["abThreshMin"] = std::to_string(ab_params.ab_thresh_min);
    params["abSumThresh"] = std::to_string(ab_params.ab_sum_thresh);

    DepthRangeParams dr_params;
    type = 4;
    status = getIniParamsImpl(&dr_params, type, config->p_tofi_cal_config);
    params["confThresh"] = std::to_string(dr_params.conf_thresh);
    params["radialThreshMin"] = std::to_string(dr_params.radial_thresh_min);
    params["radialThreshMax"] = std::to_string(dr_params.radial_thresh_max);

    JBLFConfigParams jblf_params;
    type = 2;
    status = getIniParamsImpl(&jblf_params, type, config->p_tofi_cal_config);
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
    params["headerSize"] =
        std::to_string(static_cast<float>(ir_params.headerSize));

    return status;
}

aditof::Status Adsd3500Sensor::setDepthComputeParams(
    const std::map<std::string, std::string> &params) {
    TofiConfig *config = m_bufferProcessor->getTofiCongfig();
    aditof::Status status;

    ABThresholdsParams ab_params;
    int type = 3;
    ab_params.ab_thresh_min = std::stof(params.at("abThreshMin"));
    ab_params.ab_sum_thresh = std::stof(params.at("abSumThresh"));
    status = setIniParamsImpl(&ab_params, type, config->p_tofi_cal_config);

    DepthRangeParams dr_params;
    type = 4;
    dr_params.conf_thresh = std::stof(params.at("confThresh"));
    dr_params.radial_thresh_min = std::stof(params.at("radialThreshMin"));
    dr_params.radial_thresh_max = std::stof(params.at("radialThreshMax"));
    status = setIniParamsImpl(&dr_params, type, config->p_tofi_cal_config);

    JBLFConfigParams jblf_params;
    type = 2;
    status = getIniParamsImpl(
        &jblf_params, type,
        config->p_tofi_cal_config); // get any original non-customizable value
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

    return status;
}

aditof::Status Adsd3500Sensor::waitForBufferPrivate(struct VideoDev *dev) {
    fd_set fds;
    struct timeval tv;
    int r;

    if (dev == nullptr)
        dev = &m_implData->videoDevs[0];

    FD_ZERO(&fds);
    FD_SET(dev->fd, &fds);

    tv.tv_sec = 20;
    tv.tv_usec = 0;

    r = select(dev->fd + 1, &fds, NULL, NULL, &tv);

    if (r == -1) {
        LOG(WARNING) << "select error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return aditof::Status::GENERIC_ERROR;
    } else if (r == 0) {
        LOG(WARNING) << "select timeout";
        return aditof::Status::GENERIC_ERROR;
    }
    return aditof ::Status::OK;
}

aditof::Status
Adsd3500Sensor::dequeueInternalBufferPrivate(struct v4l2_buffer &buf,
                                             struct VideoDev *dev) {
    using namespace aditof;
    Status status = Status::OK;

    if (dev == nullptr)
        dev = &m_implData->videoDevs[0];

    CLEAR(buf);
    buf.type = dev->videoBuffersType;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.length = 1;
    buf.m.planes = dev->planes;

    if (xioctl(dev->fd, VIDIOC_DQBUF, &buf) == -1) {
        LOG(WARNING) << "VIDIOC_DQBUF error "
                     << "errno: " << errno << " error: " << strerror(errno);
        switch (errno) {
        case EAGAIN:
        case EIO:
            break;
        default:
            return Status::GENERIC_ERROR;
        }
    }

    if (buf.index >= dev->nVideoBuffers) {
        LOG(WARNING) << "Not enough buffers avaialable";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status Adsd3500Sensor::getInternalBufferPrivate(
    uint8_t **buffer, uint32_t &buf_data_len, const struct v4l2_buffer &buf,
    struct VideoDev *dev) {
    if (dev == nullptr)
        dev = &m_implData->videoDevs[0];

    *buffer = static_cast<uint8_t *>(dev->videoBuffers[buf.index].start);
    buf_data_len = buf.bytesused;

    return aditof::Status::OK;
}

aditof::Status
Adsd3500Sensor::enqueueInternalBufferPrivate(struct v4l2_buffer &buf,
                                             struct VideoDev *dev) {
    if (dev == nullptr)
        dev = &m_implData->videoDevs[0];

    if (xioctl(dev->fd, VIDIOC_QBUF, &buf) == -1) {
        LOG(WARNING) << "VIDIOC_QBUF error "
                     << "errno: " << errno << " error: " << strerror(errno);
        return aditof::Status::GENERIC_ERROR;
    }

    return aditof::Status::OK;
}

aditof::Status Adsd3500Sensor::getDeviceFileDescriptor(int &fileDescriptor) {
    using namespace aditof;
    struct VideoDev *dev = &m_implData->videoDevs[0];

    if (dev->fd != -1) {
        fileDescriptor = dev->fd;
        return Status::OK;
    }

    return Status::INVALID_ARGUMENT;
}

aditof::Status Adsd3500Sensor::waitForBuffer() {

    return waitForBufferPrivate();
}

aditof::Status Adsd3500Sensor::dequeueInternalBuffer(struct v4l2_buffer &buf) {

    return dequeueInternalBufferPrivate(buf);
}

aditof::Status
Adsd3500Sensor::getInternalBuffer(uint8_t **buffer, uint32_t &buf_data_len,
                                  const struct v4l2_buffer &buf) {

    return getInternalBufferPrivate(buffer, buf_data_len, buf);
}

aditof::Status Adsd3500Sensor::enqueueInternalBuffer(struct v4l2_buffer &buf) {

    return enqueueInternalBufferPrivate(buf);
}

aditof::Status Adsd3500Sensor::writeConfigBlock(const uint32_t offset) {

    return aditof::Status::OK;
}

aditof::Status Adsd3500Sensor::queryAdsd3500() {
    using namespace aditof;
    Status status = Status::OK;
    // Ask ADSD3500 what imager is being used and whether we're using the old or new modes (CCB version)
    if (m_implData->imagerType == SensorImagerType::IMAGER_UNKNOWN ||
        m_implData->ccbVersion == CCBVersion::CCB_UNKNOWN) {

        uint8_t fwData[44] = {0};
        fwData[0] = uint8_t(1);
        adsd3500_read_payload_cmd(0x05, fwData, 44);
        if (status != Status::OK) {
            LOG(ERROR) << "Failed to retrieve fw version and git hash for "
                          "adsd3500!";
            return status;
        }
        m_implData->fw_ver = std::string((char *)(fwData), 4);

        uint16_t readValue = 0;
        int majorVersion = m_implData->fw_ver.at(0);
        if (majorVersion ==
            0) { // 0 means beta version, so version start at position 1
            majorVersion = m_implData->fw_ver.at(1);
        }
        if (majorVersion > 3) {
            status = adsd3500_read_cmd(0x0032, &readValue);
        } else {
            status = Status::GENERIC_ERROR;
        }
        if (status == aditof::Status::OK) {
            uint8_t ccb_version = readValue & 0x00FF;
            switch (ccb_version) {
            case 1: {
                m_implData->ccbVersion = CCBVersion::CCB_VERSION0;
                break;
            }
            case 2: {
                m_implData->ccbVersion = CCBVersion::CCB_VERSION1;
                break;
            }
            case 3: {
                m_implData->ccbVersion = CCBVersion::CCB_VERSION2;
                break;
            }
            default: {
                LOG(WARNING) << "Unknown CCB version read from ADSD3500: "
                             << ccb_version;
            }
            } // switch (ccb_version)

            uint8_t imager_version = (readValue & 0xFF00) >> 8;
            switch (imager_version) {
            case 1: {
                m_implData->imagerType = SensorImagerType::IMAGER_ADSD3100;
                m_modeSelector.setControl("imagerType", "adsd3100");
                break;
            }
            case 2: {
                readValue = 0;
                status = adsd3500_read_cmd(0x0115, &readValue);
                uint8_t imager_series = (readValue & 0x000F);
                if (imager_series == 1) {
                    m_implData->imagerType = SensorImagerType::IMAGER_ADSD3030;
                    m_modeSelector.setControl("imagerType", "adsd3030");
                } else if (imager_series == 2) {
                    m_implData->imagerType = SensorImagerType::IMAGER_ADTF3080;
                    m_modeSelector.setControl("imagerType", "adtf3080");
                }

                break;
            }
            default: {
                LOG(WARNING) << "Unknown imager type read from ADSD3500: "
                             << imager_version;
            }
            } // switch (imager_version)
        } else {
            status = Status::OK;
            LOG(ERROR) << "Failed to read imager type and CCB version (command "
                          "0x0032). Possibly command is not implemented on the "
                          "current adsd3500 firmware.";
            return aditof::Status::UNAVAILABLE;
        }
    }

    if (m_implData->ccbVersion != CCBVersion::CCB_UNKNOWN) {
        if (m_implData->ccbVersion == CCBVersion::CCB_VERSION0) {
            LOG(ERROR) << "Old modes are no longer supported!";
            return Status::GENERIC_ERROR;
        } else if (m_implData->ccbVersion == CCBVersion::CCB_VERSION2 &&
                   m_controls["disableCCBM"] == "0") {

            uint16_t data;
            status = adsd3500_read_cmd(0x39, &data);
            if (status != Status::OK) {
                LOG(ERROR)
                    << "Failed to check if ccb has mode map table support!";
                return status;
            }

            LOG(INFO) << "CCB master is supported. Reading mode details "
                         "from nvm.";

            m_ccbmEnabled = true;

            m_availableModes.clear();
            m_ccbmINIContent.clear();

            CcbMode modeStruct[NR_OF_MODES_FROM_CCB];
            status = adsd3500_read_payload_cmd(0x24, (uint8_t *)&modeStruct[0],
                                               SIZE_OF_MODES_FROM_CCB);
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to read mode map table from ccb!";
                return status;
            }

            for (int i = 0; i < NR_OF_MODES_FROM_CCB; i++) {
                DepthSensorModeDetails modeDetails;
                memset(&modeDetails, 0, sizeof(DepthSensorModeDetails));

                modeDetails.modeNumber = modeStruct[i].CFG_mode;
                if (modeDetails.modeNumber == 0xFF) {
                    continue;
                }

                modeDetails.baseResolutionHeight = modeStruct[i].heigth;
                modeDetails.baseResolutionWidth = modeStruct[i].width;
                modeDetails.numberOfPhases = modeStruct[i].noOfPhases;
                modeDetails.isPCM = modeStruct[i].isPCM;

                if (modeDetails.baseResolutionWidth == 0 ||
                    modeDetails.baseResolutionHeight == 0) {
                    continue;
                }

                modeDetails.frameContent.clear();
                if (!modeDetails.isPCM) {
                    modeDetails.frameContent = {"raw",  "depth", "ab",
                                                "conf", "xyz",   "metadata"};
                } else {
                    modeDetails.frameContent = {"ab", "metadata"};
                }

                //Read ini file content and store it in the sdk
                IniTableEntry iniTableContent;
                memset(&iniTableContent, 0, sizeof(IniTableEntry));
                iniTableContent.INIIndex = modeDetails.modeNumber;

                if (!modeDetails.isPCM) {
                    status = adsd3500_read_payload_cmd(
                        0x25, (uint8_t *)(&iniTableContent), 0x26);
                    if (status != Status::OK) {
                        LOG(ERROR) << "Failed to read ini content from nvm";
                        return status;
                    }

                    if (iniTableContent.INIIndex == 0xFF) {
                        LOG(INFO) << "No ini content for mode "
                                  << (int)modeDetails.modeNumber << " in nvm!";
                        continue;
                    }
                }

                iniTableContent.modeNumber = modeDetails.modeNumber;

                m_availableModes.emplace_back(modeDetails);
                m_ccbmINIContent.emplace_back(iniTableContent);
            }

        } else {
            if (m_controls["disableCCBM"] == "1") {
                LOG(INFO) << "CCB master is disabled via control. Using "
                             "sdk defined modes.";
            } else {
                LOG(INFO)
                    << "CCB master not supported. Using sdk defined modes.";
            }

            int modeToTest = 5; // We are looking at width and height for mode 5
            uint8_t tempDealiasParams[32] = {0};
            tempDealiasParams[0] = modeToTest;

            TofiXYZDealiasData tempDealiasStruct;
            uint16_t width1 = 512;
            uint16_t height1 = 512;

            uint16_t width2 = 320;
            uint16_t height2 = 256;

            // We read dealias parameters to find out the width and height for mode 5
            status = adsd3500_read_payload_cmd(0x02, tempDealiasParams, 32);
            if (status != Status::OK) {
                LOG(ERROR) << "Failed to read dealias parameters for adsd3500!";
                return status;
            }

            memcpy(&tempDealiasStruct, tempDealiasParams,
                   sizeof(TofiXYZDealiasData) - sizeof(CameraIntrinsics));

            // If mixed modes don't have accurate dimensions, switch back to simple new modes table
            if ((tempDealiasStruct.n_rows == width1 &&
                 tempDealiasStruct.n_cols == height1) ||
                (tempDealiasStruct.n_rows == width2 &&
                 tempDealiasStruct.n_cols == height2)) {
                m_modeSelector.setControl("mixedModes", "1");
            } else {
                m_modeSelector.setControl("mixedModes", "0");
            }

            //ccmb disabled. Populate struct with sdk defined variables.
            status = m_modeSelector.getAvailableModeDetails(m_availableModes);
            if (status != aditof::Status::OK) {
                LOG(ERROR) << "Failed to get available frame types for the "
                              "current configuration.";
            }
        }
    }

    if (m_implData->imagerType == SensorImagerType::IMAGER_ADSD3100) {
        status = DeviceParameters::createIniParams(
            m_iniFileStructList, m_availableModes, "adsd3100", chip_id);
    } else if (m_implData->imagerType == SensorImagerType::IMAGER_ADSD3030) {
        status = DeviceParameters::createIniParams(
            m_iniFileStructList, m_availableModes, "adsd3030", chip_id);
    } else if (m_implData->imagerType == SensorImagerType::IMAGER_ADTF3080) {
        status = DeviceParameters::createIniParams(
            m_iniFileStructList, m_availableModes, "adtf3080", chip_id);
    }
    if (status != Status::OK) {
        LOG(ERROR) << "Failed to populate ini params struct!";
        return status;
    }

    mergeIniParams(m_iniFileStructList);

    return status;
}

aditof::Status Adsd3500Sensor::adsd3500_register_interrupt_callback(
    aditof::SensorInterruptCallback &cb) {
    if (Adsd3500InterruptNotifier::getInstance().interruptsAvailable()) {
        m_interruptCallbackMap.insert({&cb, cb});
    } else {
        return aditof::Status::UNAVAILABLE;
    }

    return aditof::Status::OK;
}

aditof::Status Adsd3500Sensor::adsd3500_unregister_interrupt_callback(
    aditof::SensorInterruptCallback &cb) {

    m_interruptCallbackMap.erase(&cb);

    return aditof::Status::OK;
}

aditof::Status Adsd3500Sensor::adsd3500InterruptHandler(int signalValue) {
    uint16_t statusRegister;
    aditof::Status status = aditof::Status::OK;

    status = adsd3500_read_cmd(0x0020, &statusRegister);
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
        status = adsd3500_read_cmd(0x0038, &statusRegister);
        if (status != aditof::Status::OK) {
            LOG(ERROR) << "Failed to read imager status register!";
            return status;
        }

        m_imagerStatus = statusRegister;
        LOG(ERROR) << "Imager error detected. Error code: " << statusRegister;
    }

    for (auto m_interruptCallback : m_interruptCallbackMap) {
        m_interruptCallback.second(adsd3500Status);
    }

    if (status == Status::OK) {
        m_interruptAvailable = true;
    }

    return status;
}

aditof::Status Adsd3500Sensor::adsd3500_get_status(int &chipStatus,
                                                   int &imagerStatus) {
    using namespace aditof;
    Status status = Status::OK;

    chipStatus = m_chipStatus;
    imagerStatus = m_imagerStatus;

    return status;
}

aditof::Status
Adsd3500Sensor::setSensorConfiguration(const std::string &sensorConf) {
    aditof::Status status;
    status = m_modeSelector.setConfiguration(sensorConf);
    if (status == aditof::Status::OK) {
        LOG(INFO) << "Using sensor configuration: " << sensorConf;
    } else {
        LOG(ERROR) << "Invalid sensor configuration provided!";
    }
    return status;
}

aditof::Adsd3500Status Adsd3500Sensor::convertIdToAdsd3500Status(int status) {
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

aditof::Status Adsd3500Sensor::getIniParamsImpl(void *p_config_params,
                                                int params_group,
                                                const void *p_tofi_cal_config) {
    using namespace aditof;
    Status status = Status::OK;
    uint32_t ret;
    ret = TofiGetINIParams(p_config_params, params_group, p_tofi_cal_config);
    status = static_cast<Status>(ret);

    if (status != Status::OK) {
        LOG(ERROR) << "Failed getting ini parameters";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status Adsd3500Sensor::setIniParamsImpl(void *p_config_params,
                                                int params_group,
                                                const void *p_tofi_cal_config) {
    using namespace aditof;
    Status status = Status::OK;
    uint32_t ret;
    ret = TofiSetINIParams(p_config_params, params_group, p_tofi_cal_config);
    status = static_cast<Status>(ret);

    if (status != Status::OK) {
        LOG(ERROR) << "Failed setting ini parameters";
        return Status::GENERIC_ERROR;
    }

    return status;
}

aditof::Status Adsd3500Sensor::getDefaultIniParamsForMode(
    const std::string &imager, const std::string &mode,
    std::map<std::string, std::string> &params) {

    auto it = std::find_if(
        m_iniFileStructList.begin(), m_iniFileStructList.end(),
        [&imager, &mode](const iniFileStruct &iniF) {
            return (iniF.imagerName == imager && iniF.modeName == mode);
        });

    if (it == m_iniFileStructList.end()) {
        LOG(WARNING) << "Cannot find default parameters for imager: " << imager
                     << " and mode: " << mode;
        return aditof::Status::INVALID_ARGUMENT;
    }

    params = it->iniKeyValPairs;

    return aditof::Status::OK;
}

aditof::Status
Adsd3500Sensor::mergeIniParams(std::vector<iniFileStruct> &iniFileStructList) {

    using namespace std;
    using namespace aditof;

    if (m_ccbmEnabled) {

        for (auto &ccbmParams : m_ccbmINIContent) {

            for (auto &iniList : iniFileStructList) {

                if (iniList.modeName != "") {
                    if (ccbmParams.modeNumber ==
                        std::stoi(iniList.modeName.c_str())) {

                        iniList.iniKeyValPairs["abThreshMin"] =
                            std::to_string(ccbmParams.abThreshMin);
                        iniList.iniKeyValPairs["confThresh"] =
                            std::to_string(ccbmParams.confThresh);
                        iniList.iniKeyValPairs["radialThreshMin"] =
                            std::to_string(ccbmParams.radialThreshMin);
                        iniList.iniKeyValPairs["radialThreshMax"] =
                            std::to_string(ccbmParams.radialThreshMax);
                        iniList.iniKeyValPairs["jblfApplyFlag"] =
                            std::to_string(ccbmParams.jblfApplyFlag);
                        iniList.iniKeyValPairs["jblfWindowSize"] =
                            std::to_string(ccbmParams.jblfWindowSize);
                        iniList.iniKeyValPairs["jblfGaussianSigma"] =
                            std::to_string(ccbmParams.jblfGaussianSigma);
                        iniList.iniKeyValPairs["jblfExponentialTerm"] =
                            std::to_string(ccbmParams.jblfExponentialTerm);
                        iniList.iniKeyValPairs["jblfMaxEdge"] =
                            std::to_string(ccbmParams.jblfMaxEdge);
                        iniList.iniKeyValPairs["jblfABThreshold"] =
                            std::to_string(ccbmParams.jblfABThreshold);

                        break;
                    }
                }
            }
        }
    }

    return Status::OK;
}

aditof::Status Adsd3500Sensor::convertIniParams(iniFileStruct &iniStruct,
                                                std::string &inistr) {

    inistr = "";
    for (auto iniPairs : iniStruct.iniKeyValPairs) {
        inistr += iniPairs.first + "=" + iniPairs.second + "\n";
    }

    return Status::OK;
}

aditof::Status Adsd3500Sensor::getIniParamsArrayForMode(int mode,
                                                        std::string &iniStr) {
    std::string modestr = std::to_string(mode);
    std::string imager = "adsd3030";
    if (m_implData->imagerType == SensorImagerType::IMAGER_ADSD3100) {
        imager = "adsd3100";
    } else if (m_implData->imagerType == SensorImagerType::IMAGER_ADTF3080) {
        imager = "adtf3080";
    }

    auto it = std::find_if(
        m_iniFileStructList.begin(), m_iniFileStructList.end(),
        [&imager, &modestr](const iniFileStruct &iniF) {
            return (iniF.imagerName == imager && iniF.modeName == modestr);
        });

    if (it == m_iniFileStructList.end()) {
        LOG(WARNING) << "Cannot find default parameters for imager: " << imager
                     << " and mode: " << mode;
        return aditof::Status::INVALID_ARGUMENT;
    }

    convertIniParams(*it, iniStr);

    return Status::OK;
}