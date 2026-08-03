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

#include "aditof/rgbd_coregistration.h"
#include "aditof/log.h"

#include <cmath>
#include <cstring>
#include <fstream>
#include <json.h>

// 104 bytes = 26 × 4-byte LE floats:
//   [0-13]  RGB intrinsics: fx,fy,cx,cy,codx,cody,k1,k2,k3,k4,k5,k6,p2,p1
//   [14-25] Extrinsics rgb2tof: r11..r33, t1..t3  (translation in metres)
static constexpr std::size_t RGBD_BIN_SIZE = 104u;
static constexpr std::size_t RGBD_BIN_N_FLOATS = 26u;
static constexpr float METRES_TO_MM = 1000.0f;

namespace aditof {

// ---------------------------------------------------------------------------
// Internal JSON helper: read a double field from a json_object, return false
// if the key is missing.
// ---------------------------------------------------------------------------
static bool jsonGetDouble(json_object *parent, const char *key, float &out) {
    json_object *val = nullptr;
    if (!json_object_object_get_ex(parent, key, &val) || !val) {
        return false;
    }
    out = static_cast<float>(json_object_get_double(val));
    return true;
}

// ---------------------------------------------------------------------------
// Parse a camera_intrinsics_rgb JSON object into CameraIntrinsicsCalib.
// Field order matches both the JSON schema and the 104-byte binary blob.
// ---------------------------------------------------------------------------
static void parseRGBIntrinsics(json_object *obj, CameraIntrinsicsCalib &out) {
    jsonGetDouble(obj, "fx", out.fx);
    jsonGetDouble(obj, "fy", out.fy);
    jsonGetDouble(obj, "cx", out.cx);
    jsonGetDouble(obj, "cy", out.cy);
    jsonGetDouble(obj, "codx", out.codx);
    jsonGetDouble(obj, "cody", out.cody);
    jsonGetDouble(obj, "k1", out.k1);
    jsonGetDouble(obj, "k2", out.k2);
    jsonGetDouble(obj, "k3", out.k3);
    jsonGetDouble(obj, "k4", out.k4);
    jsonGetDouble(obj, "k5", out.k5);
    jsonGetDouble(obj, "k6", out.k6);
    jsonGetDouble(obj, "p2", out.p2);
    jsonGetDouble(obj, "p1", out.p1);
}

// ---------------------------------------------------------------------------
// Invert rgb2tof extrinsics to tof2rgb.
// Source: P_tof = R_src * P_rgb + t_src  (t_src in mm)
// Result: P_rgb = R_out * P_tof + t_out  (t_out in mm)
//   R_out = R_src^T
//   t_out = -R_src^T * t_src
// ---------------------------------------------------------------------------
static StereoExtrinsics invertRgb2Tof(const float R_src[9],
                                      const float t_src[3]) {
    StereoExtrinsics out;
    // Transpose
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            out.R[r * 3 + c] = R_src[c * 3 + r];
    // t_tof2rgb = -R^T * t_rgb2tof
    for (int r = 0; r < 3; ++r) {
        out.t[r] = 0.0f;
        for (int c = 0; c < 3; ++c)
            out.t[r] -= out.R[r * 3 + c] * t_src[c];
    }
    return out;
}

// ===========================================================================
// RGBDCoregistration implementation
// ===========================================================================

RGBDCoregistration::RGBDCoregistration() {}

Status RGBDCoregistration::setCalibration(const RGBDCalibration &calib) {
    m_calib = calib;
    m_calibLoaded = true;
    return Status::OK;
}

void RGBDCoregistration::setToFIntrinsics(
    const CameraIntrinsicsCalib &tofCalib) {
    m_calib.tof = tofCalib;
}

// ---------------------------------------------------------------------------
// Load RGB intrinsics + extrinsics from the 104-byte NVM binary blob.
// Layout: 26 x float32 LE
//   [0-13]  RGB intrinsics: fx,fy,cx,cy,codx,cody,k1,k2,k3,k4,k5,k6,p2,p1
//   [14-25] Extrinsics rgb2tof: r11,r12,r13,r21,r22,r23,r31,r32,r33,t1,t2,t3
//           Translation in metres; converted to mm and inverted to tof2rgb.
// ---------------------------------------------------------------------------
Status RGBDCoregistration::loadCalibrationFromBin(const std::string &binPath) {
    std::ifstream ifs(binPath, std::ios::binary);
    if (!ifs.good()) {
        LOG(ERROR) << "RGBD calibration binary not found: " << binPath;
        return Status::GENERIC_ERROR;
    }

    // Read exactly 104 bytes
    float f[RGBD_BIN_N_FLOATS];
    ifs.read(reinterpret_cast<char *>(f),
             static_cast<std::streamsize>(RGBD_BIN_SIZE));
    if (static_cast<std::size_t>(ifs.gcount()) != RGBD_BIN_SIZE) {
        LOG(ERROR) << "RGBD binary too small: expected " << RGBD_BIN_SIZE
                   << " bytes, got " << ifs.gcount();
        return Status::GENERIC_ERROR;
    }

    // RGB intrinsics (floats 0-13)
    m_calib.rgb.fx = f[0];
    m_calib.rgb.fy = f[1];
    m_calib.rgb.cx = f[2];
    m_calib.rgb.cy = f[3];
    m_calib.rgb.codx = f[4];
    m_calib.rgb.cody = f[5];
    m_calib.rgb.k1 = f[6];
    m_calib.rgb.k2 = f[7];
    m_calib.rgb.k3 = f[8];
    m_calib.rgb.k4 = f[9];
    m_calib.rgb.k5 = f[10];
    m_calib.rgb.k6 = f[11];
    m_calib.rgb.p2 = f[12];
    m_calib.rgb.p1 = f[13];

    // Extrinsics rgb2tof: rotation (floats 14-22) + translation in metres (23-25)
    float R_src[9], t_src_mm[3];
    for (int i = 0; i < 9; ++i)
        R_src[i] = f[14 + i];
    t_src_mm[0] = f[23] * METRES_TO_MM;
    t_src_mm[1] = f[24] * METRES_TO_MM;
    t_src_mm[2] = f[25] * METRES_TO_MM;

    // Invert to tof2rgb
    m_calib.extrinsics = invertRgb2Tof(R_src, t_src_mm);

    m_calibLoaded = true;
    LOG(INFO) << "RGBD calibration loaded from binary: " << binPath
              << "  RGB fx=" << m_calib.rgb.fx << " fy=" << m_calib.rgb.fy;
    return Status::OK;
}

// ---------------------------------------------------------------------------
// Load RGB intrinsics + extrinsics from JSON.
// Schema (matching sample_calibration.json / rgbd_calibration.json):
//   { "rgbd_calibration": {
//       "camera_intrinsics_rgb":  { fx, fy, cx, cy, codx, cody, k1..k6, p2, p1 },
//       "camera_extrinsics_rgb2tof": { r11..r33, t1, t2, t3 }  (t in metres)
//   } }
// ToF intrinsics are NOT read here; call setToFIntrinsics() separately.
// ---------------------------------------------------------------------------
Status
RGBDCoregistration::loadCalibrationFromJson(const std::string &jsonPath) {
    std::ifstream ifs(jsonPath);
    if (!ifs.good()) {
        LOG(ERROR) << "RGBD calibration file not found: " << jsonPath;
        return Status::GENERIC_ERROR;
    }

    std::string content((std::istreambuf_iterator<char>(ifs)),
                        std::istreambuf_iterator<char>());

    json_object *root = json_tokener_parse(content.c_str());
    if (!root) {
        LOG(ERROR) << "Failed to parse RGBD calibration JSON: " << jsonPath;
        return Status::GENERIC_ERROR;
    }

    // Navigate: root -> "rgbd_calibration"
    json_object *calibObj = nullptr;
    json_object_object_get_ex(root, "rgbd_calibration", &calibObj);
    if (!calibObj) {
        LOG(ERROR) << "JSON missing \"rgbd_calibration\" section: " << jsonPath;
        json_object_put(root);
        return Status::INVALID_ARGUMENT;
    }

    json_object *rgbObj = nullptr;
    json_object *extObj = nullptr;
    json_object_object_get_ex(calibObj, "camera_intrinsics_rgb", &rgbObj);
    json_object_object_get_ex(calibObj, "camera_extrinsics_rgb2tof", &extObj);

    if (!rgbObj || !extObj) {
        LOG(ERROR) << "JSON missing \"camera_intrinsics_rgb\" or "
                      "\"camera_extrinsics_rgb2tof\": "
                   << jsonPath;
        json_object_put(root);
        return Status::INVALID_ARGUMENT;
    }

    // RGB intrinsics
    parseRGBIntrinsics(rgbObj, m_calib.rgb);

    // Extrinsics rgb2tof: named fields r11..r33, t1..t3 (translation in metres)
    float R_src[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    float t_src_mm[3] = {0, 0, 0};

    jsonGetDouble(extObj, "r11", R_src[0]);
    jsonGetDouble(extObj, "r12", R_src[1]);
    jsonGetDouble(extObj, "r13", R_src[2]);
    jsonGetDouble(extObj, "r21", R_src[3]);
    jsonGetDouble(extObj, "r22", R_src[4]);
    jsonGetDouble(extObj, "r23", R_src[5]);
    jsonGetDouble(extObj, "r31", R_src[6]);
    jsonGetDouble(extObj, "r32", R_src[7]);
    jsonGetDouble(extObj, "r33", R_src[8]);

    float t1 = 0.f, t2 = 0.f, t3 = 0.f;
    jsonGetDouble(extObj, "t1", t1);
    jsonGetDouble(extObj, "t2", t2);
    jsonGetDouble(extObj, "t3", t3);
    t_src_mm[0] = t1 * METRES_TO_MM;
    t_src_mm[1] = t2 * METRES_TO_MM;
    t_src_mm[2] = t3 * METRES_TO_MM;

    // Invert to tof2rgb
    m_calib.extrinsics = invertRgb2Tof(R_src, t_src_mm);

    json_object_put(root);
    m_calibLoaded = true;

    LOG(INFO) << "RGBD calibration loaded from JSON: " << jsonPath
              << "  RGB fx=" << m_calib.rgb.fx << " fy=" << m_calib.rgb.fy;
    return Status::OK;
}

// ---------------------------------------------------------------------------
// Load from the 160-byte (40-float) GET_RGBD_CALIBRATION_DATA (0x30) response.
//
// Layout (all IEEE-754 LE float32, matching firmware doc):
//   [0 –13] ToF intrinsics   (firmware, per-mode)
//   [14–27] RGB intrinsics   (flash chunk 0x61)
//   [28–39] Extrinsics rgb2tof: r11..r33, t1..t3  (translation in metres)
// ---------------------------------------------------------------------------
Status RGBDCoregistration::loadCalibrationFrom160Bytes(const uint8_t *data,
                                                       std::size_t size) {
    static constexpr std::size_t EXPECTED = 160u;
    if (!data || size < EXPECTED) {
        LOG(ERROR) << "RGBD 0x30 response too small: expected " << EXPECTED
                   << " bytes, got " << size;
        return Status::INVALID_ARGUMENT;
    }

    // Decode 40 little-endian float32 values
    float f[40];
    for (int i = 0; i < 40; ++i) {
        std::memcpy(&f[i], data + i * 4, 4);
    }

    // Floats 0-13: ToF intrinsics (supplied by firmware for the active mode)
    m_calib.tof.fx = f[0];
    m_calib.tof.fy = f[1];
    m_calib.tof.cx = f[2];
    m_calib.tof.cy = f[3];
    m_calib.tof.codx = f[4];
    m_calib.tof.cody = f[5];
    m_calib.tof.k1 = f[6];
    m_calib.tof.k2 = f[7];
    m_calib.tof.k3 = f[8];
    m_calib.tof.k4 = f[9];
    m_calib.tof.k5 = f[10];
    m_calib.tof.k6 = f[11];
    m_calib.tof.p2 = f[12];
    m_calib.tof.p1 = f[13];

    // Floats 14-27: RGB intrinsics (from flash)
    m_calib.rgb.fx = f[14];
    m_calib.rgb.fy = f[15];
    m_calib.rgb.cx = f[16];
    m_calib.rgb.cy = f[17];
    m_calib.rgb.codx = f[18];
    m_calib.rgb.cody = f[19];
    m_calib.rgb.k1 = f[20];
    m_calib.rgb.k2 = f[21];
    m_calib.rgb.k3 = f[22];
    m_calib.rgb.k4 = f[23];
    m_calib.rgb.k5 = f[24];
    m_calib.rgb.k6 = f[25];
    m_calib.rgb.p2 = f[26];
    m_calib.rgb.p1 = f[27];

    // Floats 28-39: extrinsics rgb2tof — rotation (28-36) + translation (37-39) in metres
    float R_src[9], t_src_mm[3];
    for (int i = 0; i < 9; ++i)
        R_src[i] = f[28 + i];
    t_src_mm[0] = f[37] * METRES_TO_MM;
    t_src_mm[1] = f[38] * METRES_TO_MM;
    t_src_mm[2] = f[39] * METRES_TO_MM;

    m_calib.extrinsics = invertRgb2Tof(R_src, t_src_mm);

    m_calibLoaded = true;
    LOG(INFO) << "RGBD calibration loaded from chip (0x30 response)"
              << "  ToF fx=" << m_calib.tof.fx << "  RGB fx=" << m_calib.rgb.fx;
    return Status::OK;
}

// ---------------------------------------------------------------------------
// Save RGB intrinsics + extrinsics to a JSON file in the sample_calibration
// schema.  The stored tof2rgb extrinsics (mm) are inverted back to rgb2tof
// (m) so the output is directly usable by RGBD_CALIB_UPDATE and the Python
// coregistration example.
// ---------------------------------------------------------------------------
Status
RGBDCoregistration::saveCalibrationToJson(const std::string &jsonPath) const {
    if (!m_calibLoaded) {
        LOG(ERROR) << "saveCalibrationToJson: no calibration loaded";
        return Status::UNAVAILABLE;
    }

    // Invert stored tof2rgb → rgb2tof (same math as loading: invertRgb2Tof is self-inverse)
    StereoExtrinsics rgb2tof =
        invertRgb2Tof(m_calib.extrinsics.R, m_calib.extrinsics.t);
    // Translation: stored in mm → JSON in metres
    constexpr float MM_TO_METRES = 1.0f / METRES_TO_MM;

    // Build JSON object
    json_object *root = json_object_new_object();
    json_object *rgbd = json_object_new_object();
    json_object *rgb_intr = json_object_new_object();
    json_object *extr = json_object_new_object();

    auto addD = [](json_object *obj, const char *k, double v) {
        json_object_object_add(obj, k, json_object_new_double(v));
    };

    const auto &r = m_calib.rgb;
    addD(rgb_intr, "fx", r.fx);
    addD(rgb_intr, "fy", r.fy);
    addD(rgb_intr, "cx", r.cx);
    addD(rgb_intr, "cy", r.cy);
    addD(rgb_intr, "codx", r.codx);
    addD(rgb_intr, "cody", r.cody);
    addD(rgb_intr, "k1", r.k1);
    addD(rgb_intr, "k2", r.k2);
    addD(rgb_intr, "k3", r.k3);
    addD(rgb_intr, "k4", r.k4);
    addD(rgb_intr, "k5", r.k5);
    addD(rgb_intr, "k6", r.k6);
    addD(rgb_intr, "p2", r.p2);
    addD(rgb_intr, "p1", r.p1);

    const float *R = rgb2tof.R;
    addD(extr, "r11", R[0]);
    addD(extr, "r12", R[1]);
    addD(extr, "r13", R[2]);
    addD(extr, "r21", R[3]);
    addD(extr, "r22", R[4]);
    addD(extr, "r23", R[5]);
    addD(extr, "r31", R[6]);
    addD(extr, "r32", R[7]);
    addD(extr, "r33", R[8]);
    addD(extr, "t1", rgb2tof.t[0] * MM_TO_METRES);
    addD(extr, "t2", rgb2tof.t[1] * MM_TO_METRES);
    addD(extr, "t3", rgb2tof.t[2] * MM_TO_METRES);

    json_object_object_add(rgbd, "camera_intrinsics_rgb", rgb_intr);
    json_object_object_add(rgbd, "camera_extrinsics_rgb2tof", extr);
    json_object_object_add(root, "rgbd_calibration", rgbd);

    std::ofstream ofs(jsonPath);
    if (!ofs.good()) {
        json_object_put(root);
        LOG(ERROR) << "saveCalibrationToJson: cannot open " << jsonPath;
        return Status::GENERIC_ERROR;
    }
    ofs << json_object_to_json_string_ext(root, JSON_C_TO_STRING_PRETTY);
    json_object_put(root);

    LOG(INFO) << "RGBD calibration saved to: " << jsonPath;
    return Status::OK;
}

// ---------------------------------------------------------------------------
// Undistort a normalised ToF image point using the rational Brown-Conrady
// model (same convention as OpenCV):
//
//   r2 = x^2 + y^2
//   radial = (1 + k1*r2 + k2*r4 + k3*r6) / (1 + k4*r2 + k5*r4 + k6*r6)
//   x_u = radial*x + 2*p1*x*y + p2*(r2 + 2*x^2)
//   y_u = radial*y + p1*(r2 + 2*y^2) + 2*p2*x*y
// ---------------------------------------------------------------------------
void RGBDCoregistration::undistortToFPoint(float x_norm, float y_norm,
                                           float &x_undist,
                                           float &y_undist) const {
    const auto &K = m_calib.tof;
    // Apply centre-of-distortion offset before computing r²
    float xp = x_norm - K.codx;
    float yp = y_norm - K.cody;
    float r2 = xp * xp + yp * yp;
    float r4 = r2 * r2;
    float r6 = r4 * r2;

    float numer = 1.0f + K.k1 * r2 + K.k2 * r4 + K.k3 * r6;
    float denom = 1.0f + K.k4 * r2 + K.k5 * r4 + K.k6 * r6;
    float radial = (std::abs(denom) > 1e-6f) ? (numer / denom) : numer;

    x_undist = xp * radial + K.codx + 2.0f * K.p1 * xp * yp +
               K.p2 * (r2 + 2.0f * xp * xp);
    y_undist = yp * radial + K.cody + K.p1 * (r2 + 2.0f * yp * yp) +
               2.0f * K.p2 * xp * yp;
}

// ---------------------------------------------------------------------------
// Project a normalised 3-D direction (X/Z, Y/Z) through the RGB lens model
// and return pixel coordinates.
// ---------------------------------------------------------------------------
void RGBDCoregistration::projectToRGB(float x_norm, float y_norm, float &u_px,
                                      float &v_px) const {
    const auto &K = m_calib.rgb;
    // Apply centre-of-distortion offset before computing r²
    float xp = x_norm - K.codx;
    float yp = y_norm - K.cody;
    float r2 = xp * xp + yp * yp;
    float r4 = r2 * r2;
    float r6 = r4 * r2;

    float numer = 1.0f + K.k1 * r2 + K.k2 * r4 + K.k3 * r6;
    float denom = 1.0f + K.k4 * r2 + K.k5 * r4 + K.k6 * r6;
    float radial = (std::abs(denom) > 1e-6f) ? (numer / denom) : numer;

    float x_dist = xp * radial + K.codx + 2.0f * K.p1 * xp * yp +
                   K.p2 * (r2 + 2.0f * xp * xp);
    float y_dist = yp * radial + K.cody + K.p1 * (r2 + 2.0f * yp * yp) +
                   2.0f * K.p2 * xp * yp;

    u_px = K.fx * x_dist + K.cx;
    v_px = K.fy * y_dist + K.cy;
}

// ---------------------------------------------------------------------------
// Main coregistration entry point.
// ---------------------------------------------------------------------------
Status RGBDCoregistration::registerDepthToRGB(
    const uint16_t *depth_mm, uint32_t tofWidth, uint32_t tofHeight,
    uint16_t *registered_depth, uint32_t rgbWidth, uint32_t rgbHeight) const {

    if (!m_calibLoaded) {
        LOG(ERROR) << "RGBDCoregistration: calibration not loaded";
        return Status::UNAVAILABLE;
    }
    if (!depth_mm || !registered_depth) {
        return Status::INVALID_ARGUMENT;
    }

    // Zero-initialise output (0 == no depth data for that RGB pixel)
    std::memset(registered_depth, 0,
                static_cast<std::size_t>(rgbWidth) * rgbHeight *
                    sizeof(uint16_t));

    const auto &tofK = m_calib.tof;
    const float *R = m_calib.extrinsics.R;
    const float *t = m_calib.extrinsics.t;

    const int iRgbW = static_cast<int>(rgbWidth);
    const int iRgbH = static_cast<int>(rgbHeight);

    for (uint32_t row = 0; row < tofHeight; ++row) {
        for (uint32_t col = 0; col < tofWidth; ++col) {
            const uint16_t d = depth_mm[row * tofWidth + col];
            if (d == 0) {
                continue; // Invalid / no-return pixel
            }

            // 1. Normalise ToF pixel coordinates
            float x_norm = (static_cast<float>(col) - tofK.cx) / tofK.fx;
            float y_norm = (static_cast<float>(row) - tofK.cy) / tofK.fy;

            // 2. Undistort using ToF lens model
            float x_u, y_u;
            undistortToFPoint(x_norm, y_norm, x_u, y_u);

            // 3. Back-project to 3-D in ToF camera frame (depth in mm)
            // d is radial depth (distance from camera centre), not Z-depth
            float ray_len = std::sqrt(x_u * x_u + y_u * y_u + 1.0f);
            float Zf = static_cast<float>(d) / ray_len;
            float Xf = x_u * Zf;
            float Yf = y_u * Zf;

            // 4. Rigid transform: P_rgb = R * P_tof + t
            float Xr = R[0] * Xf + R[1] * Yf + R[2] * Zf + t[0];
            float Yr = R[3] * Xf + R[4] * Yf + R[5] * Zf + t[1];
            float Zr = R[6] * Xf + R[7] * Yf + R[8] * Zf + t[2];

            if (Zr <= 0.0f) {
                continue; // Point is behind the RGB camera
            }

            // 5. Project onto RGB image plane (apply RGB distortion)
            float u_px, v_px;
            projectToRGB(Xr / Zr, Yr / Zr, u_px, v_px);

            int u = static_cast<int>(u_px + 0.5f);
            int v = static_cast<int>(v_px + 0.5f);

            if (u < 0 || u >= iRgbW || v < 0 || v >= iRgbH) {
                continue;
            }

            // 6. Write depth; keep the nearer surface if two ToF pixels
            //    project to the same RGB pixel (z-buffer semantics).
            uint16_t &dst = registered_depth[v * rgbWidth + u];
            auto new_depth = static_cast<uint16_t>(Zr);
            if (dst == 0 || new_depth < dst) {
                dst = new_depth;
            }
        }
    }

    return Status::OK;
}

Status RGBDCoregistration::buildToFToRGBMap(
    const uint16_t *depth_mm, uint32_t tofWidth, uint32_t tofHeight,
    int32_t *rgb_u, int32_t *rgb_v, uint32_t rgbWidth,
    uint32_t rgbHeight) const {

    if (!m_calibLoaded) {
        LOG(ERROR) << "RGBDCoregistration: calibration not loaded";
        return Status::UNAVAILABLE;
    }
    if (!depth_mm || !rgb_u || !rgb_v) {
        return Status::INVALID_ARGUMENT;
    }

    const std::size_t nPixels =
        static_cast<std::size_t>(tofWidth) * tofHeight;
    std::fill(rgb_u, rgb_u + nPixels, -1);
    std::fill(rgb_v, rgb_v + nPixels, -1);

    const auto &tofK = m_calib.tof;
    const float *R   = m_calib.extrinsics.R;
    const float *t   = m_calib.extrinsics.t;
    const int iRgbW  = static_cast<int>(rgbWidth);
    const int iRgbH  = static_cast<int>(rgbHeight);

    for (uint32_t row = 0; row < tofHeight; ++row) {
        for (uint32_t col = 0; col < tofWidth; ++col) {
            const uint32_t idx = row * tofWidth + col;
            const uint16_t d   = depth_mm[idx];
            if (d == 0) continue;

            float x_norm = (static_cast<float>(col) - tofK.cx) / tofK.fx;
            float y_norm = (static_cast<float>(row) - tofK.cy) / tofK.fy;

            float x_u, y_u;
            undistortToFPoint(x_norm, y_norm, x_u, y_u);

            // d is radial depth — convert to Z-depth before back-projection
            float ray_len = std::sqrt(x_u * x_u + y_u * y_u + 1.0f);
            float Zf = static_cast<float>(d) / ray_len;
            float Xf = x_u * Zf;
            float Yf = y_u * Zf;

            float Xr = R[0]*Xf + R[1]*Yf + R[2]*Zf + t[0];
            float Yr = R[3]*Xf + R[4]*Yf + R[5]*Zf + t[1];
            float Zr = R[6]*Xf + R[7]*Yf + R[8]*Zf + t[2];

            if (Zr <= 0.0f) continue;

            float u_px, v_px;
            projectToRGB(Xr / Zr, Yr / Zr, u_px, v_px);

            int u = static_cast<int>(u_px + 0.5f);
            int v = static_cast<int>(v_px + 0.5f);

            if (u < 0 || u >= iRgbW || v < 0 || v >= iRgbH) continue;

            rgb_u[idx] = u;
            rgb_v[idx] = v;
        }
    }

    return Status::OK;
}

} // namespace aditof
