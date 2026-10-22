/*
 * BSD 3-Clause License
 *
 * Copyright (c) 2026, Analog Devices, Inc.
 * All rights reserved.
 */
#ifndef RGBD_FUSION_MANAGER_H
#define RGBD_FUSION_MANAGER_H

#include <aditof/frame.h>
#include <aditof/rgbd_coregistration.h>
#include <aditof/status_definitions.h>

#include <cstddef>
#include <cstdint>

namespace aditof {

/**
 * @brief Fuses the RGB frame with the ToF point cloud inside the camera.
 *
 * Holds the RGBD calibration for the active mode, loaded once per start().
 */
class RgbdFusionManager {
  public:
    /**
     * @brief Loads the 160-byte calibration for the active mode.
     * @param data Calibration as returned by readRGBDCalibrationFromChip()
     * @param size Must be at least 160
     * @return Status::OK on success
     */
    Status loadCalibration(const uint8_t *data, std::size_t size);

    /** @brief Drops the calibration, e.g. when none is available. */
    void reset();

    /** @return true once a calibration is loaded. */
    bool isReady() const { return m_coreg.isCalibrationLoaded(); }

    const RGBDCoregistration &coregistration() const { return m_coreg; }

    /** @brief When set, colorize() zeroes xyz points outside the RGB view. */
    void setOverlapOnly(bool enable) { m_overlapOnly = enable; }

    /**
     * @brief Fills the frame's "xyz_color" from its "xyz" and "rgb" data.
     * @param frame Frame holding xyz, rgb and xyz_color
     * @param rgbValid false when no RGB image was paired with this frame
     * @return Status::OK on success; Status::UNAVAILABLE without calibration
     *         or without the needed frame data (xyz_color is zeroed)
     */
    Status colorize(Frame *frame, bool rgbValid);

  private:
    RGBDCoregistration m_coreg;
    bool m_overlapOnly = true;
    bool m_warnedUnavailable = false;
};

} // namespace aditof

#endif // RGBD_FUSION_MANAGER_H
