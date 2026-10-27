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
#include <vector>

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

    /** @brief When set, fuse() zeroes xyz points outside the RGB view. */
    void setOverlapOnly(bool enable) { m_overlapOnly = enable; }

    /** @brief Set when the RGB image arrives rotated 180° from the calibrated one. */
    void setRgbRotated180(bool rotated);

    /**
     * @brief Sets each depth pixel's ray at a far distance (X Y Z int16 mm
     *        per pixel), which places the RGB image in the depth view.
     */
    void setPixelRays(std::vector<int16_t> rays);

    /**
     * @brief Fills the frame's "rgb" and "xyz_color".
     *
     * "rgb" is the RGB image in the depth view (depth width x height, B, G,
     * R), placed without depth; pixels outside the RGB view are 0.
     * "xyz_color" is the colour of each point of xyz.
     * @param frame Frame holding "rgb" and/or "xyz_color"
     * @param xyz Points of this frame, X Y Z int16 mm per depth pixel;
     *            nullptr when the frame has no xyz
     * @param bgr Paired RGB image (B, G, R), nullptr when none was paired
     * @param bgrWidth Width of the RGB image
     * @param bgrHeight Height of the RGB image
     * @param bgrStride Bytes per row of the RGB image
     * @return Status::OK on success; Status::UNAVAILABLE without calibration
     *         (outputs are zeroed)
     */
    Status fuse(Frame *frame, int16_t *xyz, const uint8_t *bgr,
                uint32_t bgrWidth, uint32_t bgrHeight, uint32_t bgrStride);

  private:
    RGBDCoregistration m_coreg;
    bool m_overlapOnly = true;
    bool m_rgbRotated180 = false;
    bool m_warnedUnavailable = false;
    std::vector<int16_t> m_rays;
    std::vector<float> m_rgbMap; ///< RGB position per depth pixel, from m_rays
    uint32_t m_rgbMapWidth = 0;
    uint32_t m_rgbMapHeight = 0;
};

} // namespace aditof

#endif // RGBD_FUSION_MANAGER_H
