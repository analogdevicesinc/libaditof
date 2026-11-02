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

    /** @brief Set when the RGB image arrives rotated 180° from the calibrated one. */
    void setRgbRotated180(bool rotated);

    /**
     * @brief Fills the frame's "rgb" and "xyz_color".
     *
     * Each point of xyz is projected into the RGB image with its own depth.
     * "xyz_color" is its colour and status; "rgb" is the same colour on the
     * depth grid (B, G, R), 0 where there is no depth or no RGB view.
     * With "xyz_color", xyz points outside the RGB view are zeroed.
     * @param frame Frame holding "rgb" and/or "xyz_color"
     * @param xyz Points of this frame, X Y Z int16 mm per depth pixel;
     *            nullptr when the frame has no xyz (outputs are zeroed)
     * @param bgr Paired RGB image (B, G, R), nullptr when none was paired
     * @param bgrWidth Width of the RGB image
     * @param bgrHeight Height of the RGB image
     * @param bgrStride Bytes per row of the RGB image
     * @return Status::OK on success; Status::UNAVAILABLE without calibration
     *         (outputs are zeroed)
     */
    Status fuse(Frame *frame, int16_t *xyz, const uint8_t *bgr,
                uint32_t bgrWidth, uint32_t bgrHeight, uint32_t bgrStride);

    /**
     * @brief Fills "rgb" and "xyz_color" from a recorded "rgb" image.
     *
     * A point with a recorded colour is Valid; the status of the others is
     * found again from the calibration.
     * @param frame Frame holding "rgb" and/or "xyz_color"
     * @param xyz Points of this frame, may be nullptr
     * @param bgr Recorded "rgb" image, B, G, R per depth pixel
     * @param numPoints Number of pixels in bgr
     * @param rgbWidth Width of the RGB camera image it was taken from
     * @param rgbHeight Height of the RGB camera image it was taken from
     * @return Status::OK on success; Status::INVALID_ARGUMENT when numPoints
     *         does not match the frame (outputs are zeroed);
     *         Status::UNAVAILABLE without calibration or xyz (only "rgb" is
     *         filled)
     */
    Status applyRecordedRgb(Frame *frame, int16_t *xyz, const uint8_t *bgr,
                            std::size_t numPoints, uint32_t rgbWidth,
                            uint32_t rgbHeight);

  private:
    struct Outputs {
        uint8_t *rgb = nullptr;
        uint8_t *color = nullptr;
        std::size_t rgbBytes = 0;
        std::size_t colorBytes = 0;
        std::size_t numPoints = 0;
    };
    static Outputs getOutputs(Frame *frame);
    static void clearOutputs(const Outputs &out);
    // Points' colours: the frame's "xyz_color", else m_colorScratch.
    uint8_t *colorBuffer(const Outputs &out);
    void writeOutputs(const Outputs &out, int16_t *xyz,
                      const uint8_t *rgba) const;
    RGBDCoregistration m_coreg;
    bool m_rgbRotated180 = false;
    bool m_warnedUnavailable = false;
    std::vector<uint8_t>
        m_colorScratch; ///< point colours when the frame has no xyz_color
};

} // namespace aditof

#endif // RGBD_FUSION_MANAGER_H
