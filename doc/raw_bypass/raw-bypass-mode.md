# Raw Bypass Mode Implementation

## Overview

Raw bypass mode enables the ADCAM Camera Kit to capture unprocessed 12-bit Bayer raw sensor data directly from the ADTF3175D image sensor, bypassing the ToF depth computation pipeline. This mode is useful for:

- Sensor characterization and validation
- Custom image processing algorithm development
- Raw data collection for training/testing
- Debugging sensor-level issues

## Architecture

### Data Flow Comparison

**Standard ToF Mode:**
```
ADTF3175D Sensor → ADSD3500 ISP (depth compute) → V4L2 Driver → BufferProcessor
→ TofiCompute() → Depth + AB + Confidence → Application
```

**Raw Bypass Mode:**
```
ADTF3175D Sensor → ADSD3500 ISP (raw passthrough) → V4L2 Driver → BufferProcessor
→ Direct memcpy → Raw Bayer data → Application
```

### Key Differences

| Aspect | Standard Mode | Raw Bypass Mode |
|--------|--------------|-----------------|
| **Frame Content** | Depth, AB, Confidence | Raw Bayer (RG12) |
| **ISP Processing** | Multi-phase ToF computation | Raw sensor passthrough |
| **Buffer Size** | 1024×1024 (base resolution) | 2048×3072 (full sensor) |
| **Bytes per Pixel** | 2 (depth) + 2 (AB) + 4 (conf) | 2 (unpacked 12-bit) |
| **Frame Size (Mode 0)** | ~2.1 MB | ~13 MB |
| **Number of Phases** | 2 or 3 | 1 (single frame) |
| **ToFi Library** | Required | Skipped |
| **Depth/AB/Conf Bits** | 12-16 bits | All 0 |

## Configuration

### Enable Raw Bypass Mode

Create or modify `update.json` configuration file:

```json
{
  "mode0": {
    "rawBypassMode": "1"
  },
  "mode1": {
    "rawBypassMode": "1"
  }
}
```

### Capture Raw Frames

```bash
cd build/examples/data_collect
./data_collect --m 0 --n 10 --f raw_capture --lcf update.json
```

Output files will be saved in `media/` directory with `.adcam` extension containing raw Bayer data.

## Implementation Details

### 1. Driver Configuration Table

**File:** `libaditof/sdk/src/connections/target/sensor-tables/driver_configuration_table.h`

Added new driver configuration table for raw bypass modes:

```cpp
const std::vector<DriverConfiguration> m_adsd3500rawBypass = {
    // Mode 0: MP raw frame - 2048×3072 pixels
    {"1024", "1024", "1", "0", "0", "0", "RG12", 2048, 3072, 1},
    
    // Mode 1: MP raw frame - 2048×3072 pixels
    {"1024", "1024", "1", "0", "0", "0", "RG12", 2048, 3072, 1},
    
    // Modes 2-6: QMP raw frames - 1024×4608 pixels
    {"512", "512", "1", "0", "0", "0", "RG12", 1024, 4608, 1},
    // ... (modes 3-6 similar)
};
```

**Configuration Fields:**
- `baseWidth/baseHeight`: Base resolution (1024×1024 or 512×512)
- `phases`: "1" (single frame, no multi-phase ToF)
- `depthBits/abBits/confBits`: "0" (no ToF processing)
- `pixelFormat`: "RG12" (unpacked 12-bit Bayer in 16-bit container)
- `driverWidth/driverHeight`: Full sensor dimensions in pixels
- `pixelFormatIndex`: 1 (indicates RG12 format)

### 2. Mode Selection Logic

**File:** `libaditof/sdk/src/connections/target/adsd3500_mode_selector.cpp`

**Function:** `updateConfigurationTable()`

```cpp
// Check for raw bypass mode
if (m_controls["rawBypassMode"] == "1") {
    // Search raw bypass table
    for (auto driverConf : m_adsd3500rawBypass) {
        if (/* resolution match */) {
            // Convert pixel dimensions to bytes (RG12: 2 bytes/pixel)
            int bytesPerPixel = (driverConf.pixelFormatIndex == 1) ? 2 : 1;
            configurationTable.frameWidthInBytes = driverConf.driverWidth * bytesPerPixel;
            configurationTable.frameHeightInBytes = driverConf.driverHeigth;
            configurationTable.pixelFormatIndex = driverConf.pixelFormatIndex;
            configurationTable.isRawBypass = 1;  // Set flag for downstream
            configurationTable.numberOfPhases = 1;
            configurationTable.frameContent = {"raw"};
            return aditof::Status::OK;
        }
    }
}
// Otherwise, use standard depth mode table
```

**Why multiply by 2?**
- V4L2 buffer allocation requires byte dimensions
- RG12 format stores 12-bit data in 16-bit containers (2 bytes per pixel)
- `frameWidthInBytes` = `driverWidth` (pixels) × 2 bytes/pixel

### 3. V4L2 Format Configuration

**File:** `libaditof/sdk/src/connections/target/adsd3500_sensor.cpp`

**Function:** `setFrameType()`

```cpp
// Set V4L2 format
fmt.fmt.pix.pixelformat = pixelFormat;

// V4L2 expects pixel dimensions, not byte dimensions
if (type.isRawBypass) {
    // Raw bypass: frameWidthInBytes is in bytes, convert back to pixels
    fmt.fmt.pix.width = type.frameWidthInBytes / 2;
} else {
    // Standard modes: frameWidthInBytes already in pixels
    fmt.fmt.pix.width = type.frameWidthInBytes;
}
fmt.fmt.pix.height = type.frameHeightInBytes;
```

**Critical:** V4L2 driver interprets `width` field as pixel count, not byte count.

**Dimension Flow:**
```
driver_configuration_table.h:  driverWidth = 2048 pixels
                                      ↓
adsd3500_mode_selector.cpp:    frameWidthInBytes = 2048 × 2 = 4096 bytes
                                      ↓
adsd3500_sensor.cpp:           fmt.fmt.pix.width = 4096 / 2 = 2048 pixels
                                      ↓
V4L2 Driver:                   Delivers 2048×3072 pixels × 2 bytes = 12,582,912 bytes
                                      ↓
Buffer (with NVIDIA padding):  12,587,008 bytes total
```

### 4. Buffer Processing

**File:** `libaditof/sdk/src/connections/target/buffer_processor.cpp`

#### 4.1 Frame Size Calculation

**Function:** `calculateFrameSize()`

```cpp
// Use driver dimensions for raw bypass, output dimensions for standard
uint32_t width = m_isRawBypassMode ? m_driverFrameWidth : m_outputFrameWidth;
uint32_t height = m_isRawBypassMode ? m_driverFrameHeight : m_outputFrameHeight;

if (m_isRawBypassMode) {
    // Raw frames: single buffer with full sensor data
    // No separate AB/Confidence buffers
    uint32_t rawBufferSizeInUint16 = (width * height) + (width);  // NVIDIA alignment
    m_rawFrameBufferSize = rawBufferSizeInUint16 * sizeof(uint16_t);
    
    if (rawBufferSizeInUint16 > m_tofiBufferSize) {
        m_tofiBufferSize = rawBufferSizeInUint16;
    }
} else {
    // Standard mode: separate depth, AB, confidence buffers
    // ... (existing logic)
}
```

**NVIDIA Platform Alignment:**
```
Raw buffer size = (width × height + width) × sizeof(uint16_t)
                = (4096 × 3072 + 4096) × 2
                = 12,587,008 bytes
```

Extra width accounts for NVIDIA platform's line alignment padding.

#### 4.2 Frame Processing

**Function:** `processThread()`

```cpp
if (m_isRawBypassMode) {
    // Raw bypass: direct copy, no ToFi processing
    const size_t bufferSizeBytes = process_frame.size;
    const size_t maxBufferSize = m_rawFrameBufferSize;
    
    // Validate buffer size
    if (bufferSizeBytes > maxBufferSize) {
        LOG(ERROR) << "Buffer size mismatch: " << bufferSizeBytes 
                   << " > " << maxBufferSize;
        // ... error handling
    }
    
    // Direct memcpy from V4L2 buffer to output buffer
    memcpy(tofi_compute_io_buff.get(), process_frame.data.get(), bufferSizeBytes);
    
    // Write to file if recording
    if (m_state == ST_RECORD && m_stream_file_out.is_open()) {
        writeFrame((uint8_t *)tofi_compute_io_buff.get(), bufferSizeBytes);
    }
    
    // Push to output queue
    m_process_done_Q.push(tofi_compute_io_buff);
} else {
    // Standard ToF processing with TofiCompute()
    // ... (existing logic)
}
```

### 5. Camera Initialization

**File:** `libaditof/sdk/src/cameras/itof-camera/camera_itof.cpp`

**Function:** `setMode()`

```cpp
// Skip ToFi initialization for raw bypass mode
if (!m_pcmFrame && !m_modeDetailsCache.isRawBypass) {
    // Initialize ToFi compute library with INI and calibration data
    // ... (ToFi initialization)
}
```

**Why skip ToFi?**
- Raw bypass doesn't perform depth computation
- ToFi library expects valid depth/AB/conf bit configurations
- Raw mode uses all-zero bits (triggers "Invalid bits combination" error)

#### Recording with Correct Frame Content

```cpp
// Use updated frameContent from mode cache
for (const auto &item : m_modeDetailsCache.frameContent) {
    FrameDataDetails fDataDetails;
    fDataDetails.type = item;
    fDataDetails.width = m_modeDetailsCache.frameWidthInBytes;
    fDataDetails.height = m_modeDetailsCache.frameHeightInBytes;
    // ...
}
```

Changed from `(*modeIt).frameContent` to `m_modeDetailsCache.frameContent` to use the updated "raw" content type set by mode selector.

## Pixel Format Details

### RG12 (V4L2_PIX_FMT_SRGGB12)

- **Format ID:** 842090322 (0x32314752)
- **Bayer Pattern:** RGGB (Red-Green-Green-Blue)
- **Bit Depth:** 12 bits per pixel
- **Storage:** Unpacked (each pixel in 16-bit container)
- **Byte Order:** Little-endian
- **Bits per Pixel:** 16 (12 significant + 4 padding)

### Memory Layout

```
Pixel Data (16-bit per pixel):
[R0][G1][R2][G3]...  ← Row 0 (even)
[G0][B1][G2][B3]...  ← Row 1 (odd)
[R0][G1][R2][G3]...  ← Row 2 (even)
...

Each uint16_t contains:
Bits [15:12] = 0 (padding)
Bits [11:0]  = 12-bit pixel value
```

## Buffer Size Calculations

### Mode 0 (MP - 2048×3072 pixels)

```
Pixel dimensions:     2048 × 3072 = 6,291,456 pixels
Raw buffer (ideal):   6,291,456 × 2 bytes = 12,582,912 bytes
NVIDIA alignment:     +4,096 bytes (extra line)
Actual buffer:        12,587,008 bytes
File size:            12,587,008 + 664 (header) = 12,587,672 bytes (~13 MB)
```

### Mode 2-6 (QMP - 1024×4608 pixels)

```
Pixel dimensions:     1024 × 4608 = 4,718,592 pixels
Raw buffer (ideal):   4,718,592 × 2 bytes = 9,437,184 bytes
NVIDIA alignment:     +2,048 bytes (extra line)
Actual buffer:        9,439,232 bytes
File size:            ~9.4 MB
```

## Testing and Validation

### Verify Raw Capture Works

```bash
# Build project
cd build
cmake --build . --target data_collect -j$(nproc)

# Run raw capture test
cd examples/data_collect
./data_collect --m 0 --n 1 --f raw_test --lcf update.json

# Verify file size (should be ~13 MB for mode 0)
ls -lh media/*.adcam
```

Expected output:
```
-rw-rw-r-- 1 analog analog 13M Mar 10 16:31 media/aditof_20260310_110137_*.adcam
```

### Validate Raw Data

Use Python bindings to read and verify raw frames:

```python
import aditof
import numpy as np

# Open camera in raw bypass mode
system = aditof.System()
cameras = system.getCameraList()
camera = cameras[0]
camera.initialize()

# Load configuration with rawBypassMode=1
camera.setControl("rawBypassMode", "1")
camera.setFrameType("raw")

# Capture frame
frame = camera.requestFrame()
raw_data = np.array(frame.getData("raw"), copy=False)

# Verify dimensions
print(f"Raw frame shape: {raw_data.shape}")  # Should be (3072, 4096) bytes
print(f"Data type: {raw_data.dtype}")         # Should be uint8 or uint16
print(f"Min/Max values: {raw_data.min()}, {raw_data.max()}")
```

## Troubleshooting

### Issue: Segmentation Fault

**Symptom:** Application crashes immediately after mode switch.

**Cause:** Frame content mismatch between mode table and camera expectations.

**Solution:** Ensure `frameContent` is set to `"raw"` consistently in mode selector and camera code.

### Issue: Wrong File Size (514 KB instead of 13 MB)

**Symptom:** Captured files are much smaller than expected.

**Cause:** V4L2 format width set incorrectly (byte dimensions passed as pixel dimensions).

**Solution:** Ensure `adsd3500_sensor.cpp` divides `frameWidthInBytes` by 2 for raw bypass mode:

```cpp
if (type.isRawBypass) {
    fmt.fmt.pix.width = type.frameWidthInBytes / 2;  // Convert bytes to pixels
}
```

### Issue: Buffer Size Mismatch Error

**Symptom:** Log shows "Buffer size mismatch: X > Y" errors.

**Cause:** Buffer allocation doesn't account for NVIDIA platform alignment.

**Solution:** Ensure `calculateFrameSize()` includes extra width for alignment:

```cpp
uint32_t rawBufferSizeInUint16 = (width * height) + (width);  // +width for alignment
```

### Issue: Standard Modes Broken After Changes

**Symptom:** Normal depth modes fail or produce wrong dimensions.

**Cause:** Width conversion applied unconditionally to all modes.

**Solution:** Make V4L2 width conversion conditional on `isRawBypass` flag:

```cpp
if (type.isRawBypass) {
    fmt.fmt.pix.width = type.frameWidthInBytes / 2;
} else {
    fmt.fmt.pix.width = type.frameWidthInBytes;  // Standard modes
}
```

## Performance Considerations

### Memory Usage

Raw bypass mode requires significantly more memory per frame:

| Mode | Resolution | Buffer Size | Frames/Second | Memory Bandwidth |
|------|-----------|-------------|---------------|------------------|
| Standard MP (mode 0) | 1024×1024 | ~2.1 MB | 30 fps | ~63 MB/s |
| Raw MP (mode 0) | 2048×3072 | ~13 MB | 30 fps | ~390 MB/s |
| Standard QMP (mode 2) | 512×512 | ~0.5 MB | 30 fps | ~15 MB/s |
| Raw QMP (mode 2) | 1024×4608 | ~9.4 MB | 30 fps | ~282 MB/s |

### Processing Time

- **Raw bypass:** ~minimal overhead (single memcpy)
- **Standard ToF:** ToFi computation adds 10-20ms per frame

### Storage Requirements

For 1000 frames:
- Standard mode: ~2 GB
- Raw bypass mode: ~13 GB

## Future Enhancements

### Potential Improvements

1. **8-bit Raw Mode Support**
   - Add configuration for packed 8-bit raw (1 byte/pixel)
   - Reduce file size by ~50%

2. **Compression**
   - Add lossless compression for raw frames
   - Reduce storage requirements

3. **Region of Interest (ROI)**
   - Capture partial sensor area for lower bandwidth

4. **Multi-Frame Averaging**
   - Average multiple raw frames to reduce noise

5. **Live Preview**
   - Add debayering and display for raw frames in viewer applications

## References

- **V4L2 Documentation:** [Linux Media Subsystem](https://www.kernel.org/doc/html/latest/userspace-api/media/v4l/v4l2.html)
- **ADTF3175D Datasheet:** Analog Devices Time-of-Flight Sensor
- **ADSD3500 Datasheet:** Dual Depth ISP Processor
- **Bayer Pattern:** [Wikipedia - Bayer Filter](https://en.wikipedia.org/wiki/Bayer_filter)

## Version History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2026-03-10 | ADCAM Team | Initial implementation of raw bypass mode with RG12 support |

---

**Note:** This implementation is specific to the NVIDIA Jetson Orin Nano platform with ADSD3500 dual ISP configuration. Buffer size calculations include platform-specific alignment requirements.
