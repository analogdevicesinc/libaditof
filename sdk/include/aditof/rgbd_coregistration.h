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
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef RGBD_COREGISTRATION_H
#define RGBD_COREGISTRATION_H

#include "aditof/sdk_exports.h"
#include "aditof/status_definitions.h"
#include <cstdint>
#include <string>

/**
 * @file rgbd_coregistration.h
 * @brief RGB-Depth coregistration: maps ToF depth pixels into RGB camera space.
 *
 * Algorithm overview
 * ==================
 * For each valid ToF pixel (u, v) with depth d (mm):
 *  1. Normalize:  x = (u - cx_tof) / fx_tof,  y = (v - cy_tof) / fy_tof
 *  2. Undistort using the Brown-Conrady model (k1–k6, p1, p2, codx, cody)
 *  3. Back-project to 3-D:  P_tof = [x_u * d,  y_u * d,  d]
 *  4. Transform:  P_rgb = R * P_tof + t   (tof2rgb extrinsics, t in mm)
 *  5. Project onto RGB image with optional RGB lens distortion
 *  6. Write d (mm) into registered_depth at the projected RGB pixel; nearest
 *     depth wins when multiple ToF pixels map to the same RGB pixel.
 *
 * Calibration sources
 * ===================
 * - ToF intrinsics: read from ADSD3500 firmware via adsd3500_read_payload_cmd(0x01)
 *   and set via setToFIntrinsics().
 * - RGB intrinsics + extrinsics: stored in NVM as a 104-byte binary blob.
 *   Load via loadCalibrationFromBin() or loadCalibrationFromJson().
 *
 * Binary format (104 bytes = 26 × 4-byte IEEE 754 LE floats)
 * ===========================================================
 *   [0 –13]  RGB intrinsics: fx, fy, cx, cy, codx, cody, k1, k2, k3, k4, k5, k6, p2, p1
 *   [14–25]  Extrinsics (rgb2tof): r11, r12, r13, r21, r22, r23, r31, r32, r33, t1, t2, t3
 *            Translation in metres; internally converted to mm and inverted to tof2rgb.
 *
 * JSON format
 * ===========
 * {
 *   "rgbd_calibration": {
 *     "camera_intrinsics_rgb": {
 *       "fx": 1221.0, "fy": 1221.0, "cx": 960.0, "cy": 600.0,
 *       "codx": 0.0,  "cody": 0.0,
 *       "k1": 0.0, "k2": 0.0, "k3": 0.0, "k4": 0.0, "k5": 0.0, "k6": 0.0,
 *       "p2": 0.0, "p1": 0.0
 *     },
 *     "camera_extrinsics_rgb2tof": {
 *       "r11":1.0,"r12":0.0,"r13":0.0,
 *       "r21":0.0,"r22":1.0,"r23":0.0,
 *       "r31":0.0,"r32":0.0,"r33":1.0,
 *       "t1":0.0, "t2":0.0, "t3":0.0
 *     }
 *   }
 * }
 * Translation in metres; internally converted to mm and inverted to tof2rgb.
 */

namespace aditof {

/**
 * @brief Intrinsic parameters for one camera (ToF or RGB).
 *
 * Field order matches the ADSD3500 ISP payload (command 0x01) and the
 * 104-byte NVM binary blob produced by rgbd_json_to_bin.py:
 *   fx, fy, cx, cy, codx, cody, k1, k2, k3, k4, k5, k6, p2, p1
 *
 * Distortion follows the rational Brown-Conrady model:
 *   r² = (x-codx)² + (y-cody)²
 *   radial  = (1 + k1*r² + k2*r⁴ + k3*r⁶) / (1 + k4*r² + k5*r⁴ + k6*r⁶)
 *   tangential corrections via p1, p2
 */
struct CameraIntrinsicsCalib {
    float fx = 0.f;   ///< Horizontal focal length (pixels)
    float fy = 0.f;   ///< Vertical focal length (pixels)
    float cx = 0.f;   ///< Principal point X (pixels)
    float cy = 0.f;   ///< Principal point Y (pixels)
    float codx = 0.f; ///< Centre-of-distortion offset X (pixels, usually 0)
    float cody = 0.f; ///< Centre-of-distortion offset Y (pixels, usually 0)
    float k1 = 0.f;   ///< Radial distortion coefficient 1
    float k2 = 0.f;   ///< Radial distortion coefficient 2
    float k3 = 0.f;   ///< Radial distortion coefficient 3
    float k4 = 0.f;   ///< Rational distortion denominator 4
    float k5 = 0.f;   ///< Rational distortion denominator 5
    float k6 = 0.f;   ///< Rational distortion denominator 6
    float p2 = 0.f;   ///< Tangential distortion coefficient 2
    float p1 = 0.f;   ///< Tangential distortion coefficient 1
};

/**
 * @brief Stereo extrinsic parameters stored as tof→rgb transform.
 *
 * P_rgb = R * P_tof + t  (t in mm).
 * R is stored row-major: {r11, r12, r13, r21, r22, r23, r31, r32, r33}.
 * When loaded from the binary/JSON (which stores rgb2tof), the inverse is
 * computed automatically so this struct always holds the tof2rgb direction.
 */
struct StereoExtrinsics {
    float R[9] = {1, 0, 0, 0, 1,
                  0, 0, 0, 1}; ///< 3×3 rotation tof→rgb (row-major)
    float t[3] = {0, 0, 0};    ///< Translation tof→rgb (mm)
};

/**
 * @brief Complete RGB-Depth calibration bundle.
 */
struct RGBDCalibration {
    CameraIntrinsicsCalib tof;   ///< ToF camera intrinsics
    CameraIntrinsicsCalib rgb;   ///< RGB camera intrinsics
    StereoExtrinsics extrinsics; ///< Stereo transform (ToF → RGB)
};

/**
 * @class RGBDCoregistration
 * @brief Maps a ToF depth frame into RGB camera image space.
 *
 * Usage:
 * @code
 *   aditof::RGBDCoregistration coreg;
 *   // RGB intrinsics + extrinsics from NVM binary:
 *   coreg.loadCalibrationFromBin("/path/to/rgbd_calibration.bin");
 *   // ToF intrinsics from ISP (CameraIntrinsics → CameraIntrinsicsCalib):
 *   coreg.setToFIntrinsics(tofCalib);
 *
 *   std::vector<uint16_t> registered(rgbW * rgbH, 0);
 *   coreg.registerDepthToRGB(depth.data(), tofW, tofH,
 *                            registered.data(), rgbW, rgbH);
 * @endcode
 *
 * Thread safety: const methods (registerDepthToRGB) are safe to call
 * concurrently from multiple threads once calibration is loaded.
 */
class SDK_API RGBDCoregistration {
  public:
    RGBDCoregistration();
    ~RGBDCoregistration() = default;

    // Non-copyable, movable
    RGBDCoregistration(const RGBDCoregistration &) = delete;
    RGBDCoregistration &operator=(const RGBDCoregistration &) = delete;
    RGBDCoregistration(RGBDCoregistration &&) = default;
    RGBDCoregistration &operator=(RGBDCoregistration &&) = default;

    /**
     * @brief Load RGB intrinsics + extrinsics from the 104-byte NVM binary.
     *
     * Binary layout (26 × 4-byte LE floats):
     *   [0–13]  RGB intrinsics: fx,fy,cx,cy,codx,cody,k1–k6,p2,p1
     *   [14–25] Extrinsics (rgb2tof): r11–r33, t1–t3 (translation in metres)
     * Extrinsics are inverted to tof2rgb and translation converted to mm.
     * ToF intrinsics must be set separately via setToFIntrinsics().
     *
     * @param binPath Path to the 104-byte binary calibration file.
     * @return Status::OK on success.
     */
    Status loadCalibrationFromBin(const std::string &binPath);

    /**
     * @brief Load calibration from the 160-byte GET_RGBD_CALIBRATION_DATA
     *        (command 0x30) chip response.
     *
     * Layout: 40 × 4-byte LE float32
     *   [0 –13] ToF intrinsics  (from firmware, per active mode)
     *   [14–27] RGB intrinsics  (from flash chunk 0x61)
     *   [28–39] Extrinsics rgb2tof — rotation (row-major) then translation (m)
     *
     * Both the ToF and RGB intrinsics are stored; the separate
     * setToFIntrinsics() call is therefore NOT needed after this.
     * Translation is converted internally from metres to mm and the
     * rgb2tof extrinsic is inverted to tof2rgb.
     *
     * @param data  Pointer to the 160-byte response buffer.
     * @param size  Must be >= 160.
     * @return Status::OK on success; Status::INVALID_ARGUMENT if data is
     *         null or size < 160.
     */
    Status loadCalibrationFrom160Bytes(const uint8_t *data, std::size_t size);

    /**
     * @brief Load RGB intrinsics + extrinsics from a JSON file on disk.
     *
     * Expected schema — see header comment for full structure.
     * ToF intrinsics must be set separately via setToFIntrinsics().
     *
     * @param jsonPath Path to the calibration JSON file.
     * @return Status::OK on success.
     */
    Status loadCalibrationFromJson(const std::string &jsonPath);

    /**
     * @brief Set ToF camera intrinsics (obtained from ADSD3500 ISP).
     *
     * Call this after loadCalibrationFromBin()/loadCalibrationFromJson() to
     * supply the ToF intrinsics read via adsd3500_read_payload_cmd(0x01).
     * The CameraIntrinsicsCalib field order matches CameraIntrinsics from
     * tofi_camera_intrinsics.h exactly.
     *
     * @param tofCalib ToF intrinsics (fx,fy,cx,cy,codx,cody,k1–k6,p2,p1).
     */
    void setToFIntrinsics(const CameraIntrinsicsCalib &tofCalib);

    /**
     * @brief Set calibration directly from a fully populated struct.
     * @param calib Calibration data (extrinsics must be tof2rgb, t in mm).
     * @return Status::OK always.
     */
    Status setCalibration(const RGBDCalibration &calib);

    /**
     * @brief Project the ToF depth map into the RGB camera plane.
     *
     * @param[in]  depth_mm         ToF depth image, row-major, uint16 mm
     *                              (0 = invalid pixel).
     * @param[in]  tofWidth         Width of the ToF frame.
     * @param[in]  tofHeight        Height of the ToF frame.
     * @param[out] registered_depth Output depth at RGB resolution, row-major,
     *                              uint16 mm (0 = no ToF data). Must be
     *                              pre-allocated: rgbWidth × rgbHeight uint16_t.
     * @param[in]  rgbWidth         Width of the target RGB frame.
     * @param[in]  rgbHeight        Height of the target RGB frame.
     * @return Status::OK on success.
     *         Status::UNAVAILABLE if calibration has not been loaded.
     *         Status::INVALID_ARGUMENT if any pointer is null.
     */
    Status registerDepthToRGB(const uint16_t *depth_mm, uint32_t tofWidth,
                              uint32_t tofHeight, uint16_t *registered_depth,
                              uint32_t rgbWidth, uint32_t rgbHeight) const;

    /**
     * @brief Build a per-ToF-pixel lookup table mapping each ToF pixel to its
     *        corresponding (u, v) coordinate in the RGB image plane.
     *
     * For each ToF pixel (col, row) with depth d > 0, applies the full
     * tof→rgb geometric transform and writes the projected RGB pixel
     * coordinates into rgb_u and rgb_v.  Pixels that are invalid (d == 0),
     * behind the RGB camera, or project outside [0, rgbWidth) × [0, rgbHeight)
     * are written as -1.
     *
     * Output arrays must be pre-allocated to tofWidth × tofHeight elements.
     *
     * @param depth_mm   ToF depth image, row-major, uint16 mm (0 = invalid).
     * @param tofWidth   Width of the ToF frame.
     * @param tofHeight  Height of the ToF frame.
     * @param rgb_u      Output u (column) in RGB space per ToF pixel, or -1.
     * @param rgb_v      Output v (row)    in RGB space per ToF pixel, or -1.
     * @param rgbWidth   Width of the RGB frame.
     * @param rgbHeight  Height of the RGB frame.
     * @return Status::OK on success; Status::UNAVAILABLE if calibration not
     *         loaded; Status::INVALID_ARGUMENT if any pointer is null.
     */
    Status buildToFToRGBMap(const uint16_t *depth_mm, uint32_t tofWidth,
                            uint32_t tofHeight, int32_t *rgb_u, int32_t *rgb_v,
                            uint32_t rgbWidth, uint32_t rgbHeight) const;

    /** @return true once a calibration has been successfully loaded. */
    bool isCalibrationLoaded() const { return m_calibLoaded; }

    /** @return Reference to the active calibration (valid after load). */
    const RGBDCalibration &getCalibration() const { return m_calib; }

    /**
     * @brief Save the loaded calibration to a JSON file.
     *
     * Writes the RGB intrinsics and extrinsics in the sample_calibration.json
     * schema (camera_intrinsics_rgb + camera_extrinsics_rgb2tof, translation
     * in metres).  The stored tof2rgb extrinsics are inverted back to rgb2tof
     * before writing so the file is usable directly with RGBD_CALIB_UPDATE
     * and the Python coregistration example.
     *
     * @param jsonPath Path to write (created or overwritten).
     * @return Status::OK on success; Status::UNAVAILABLE if no calibration
     *         is loaded; Status::GENERIC_ERROR if the file cannot be written.
     */
    Status saveCalibrationToJson(const std::string &jsonPath) const;

  private:
    /**
     * @brief Undistort a normalised ToF image point in-place.
     *
     * Applies the rational Brown-Conrady undistortion model using the ToF
     * intrinsics stored in m_calib.tof.
     *
     * @param[in]  x_norm   Normalized x coordinate: (u - cx) / fx
     * @param[in]  y_norm   Normalized y coordinate: (v - cy) / fy
     * @param[out] x_undist Undistorted normalised x
     * @param[out] y_undist Undistorted normalised y
     */
    void undistortToFPoint(float x_norm, float y_norm, float &x_undist,
                           float &y_undist) const;

    /**
     * @brief Project a normalised 3-D ray onto the RGB image with distortion.
     *
     * @param[in]  x_norm  X / Z in RGB camera space
     * @param[in]  y_norm  Y / Z in RGB camera space
     * @param[out] u_px    Distorted pixel column in the RGB image
     * @param[out] v_px    Distorted pixel row    in the RGB image
     */
    void projectToRGB(float x_norm, float y_norm, float &u_px,
                      float &v_px) const;

    RGBDCalibration m_calib;
    bool m_calibLoaded = false;
};

} // namespace aditof

#endif // RGBD_COREGISTRATION_H
