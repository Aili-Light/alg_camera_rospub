# Design: Validate Topics — Compressed Image Support and IMU Text Output

**Date:** 2026-07-12
**Status:** Draft
**Package:** `alg_camera_rospub`

## Motivation

`scripts/validate_topics.py` currently only decodes raw `sensor_msgs/msg/Image` messages with `mono8` encoding. The publisher nodes (`single_camera_node.cpp`, `stereo_camera_node.cpp`) were recently updated to publish `sensor_msgs/msg/CompressedImage` (JPEG) as an alternative output mode. The validation script needs to handle both message types transparently.

Additionally, IMU data is only printed to console and lost after the script exits. Saving it to a text file enables offline analysis.

## Goals

- Auto-detect whether the camera topic carries `Image` or `CompressedImage`, subscribe accordingly
- Decode compressed JPEG messages via `cv2.imdecode`
- Expand raw image encoding support beyond `mono8` to handle color formats (BGR, RGB, YUV variants)
- Save IMU data (gyro + accelerometer) to a space-delimited text file with nanosecond timestamps
- No new CLI arguments — behavior change is transparent
- Existing behavior (images saved as JPEG, console output, statistics) unchanged

## Non-Goals

- Streaming multiple camera topics simultaneously
- Support for image encodings beyond those listed below
- Configurable IMU output format (CSV, binary, etc.)

## Design

All changes are in a single file: `scripts/validate_topics.py`.

### 1. Compressed image auto-detection

On startup, after creating the subscriber node, call `get_topic_names_and_types()` to inspect the camera topic. Map the result to a message type:

| Type string found | Subscribe to | Decode method |
|---|---|---|
| `sensor_msgs/msg/CompressedImage` | `CompressedImage` | `cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)` |
| `sensor_msgs/msg/Image` | `Image` | Existing encoding dispatch (see §2) |
| Topic not found | — | Log warning, retry every 1s up to 5 times, then raise error |

The subscription is created with the resolved message type. The callback signature adapts accordingly — either `_on_image(msg: Image)` or `_on_compressed_image(msg: CompressedImage)`.

### 2. Raw image encoding dispatch

The existing `_on_image` handler currently checks only for `mono8`. Replace with a dispatch table:

| `msg.encoding` | OpenCV constant | Pre-save color conversion |
|---|---|---|
| `mono8` | — | None (grayscale JPEG as-is) |
| `bgr8` | — | None (already BGR, `imwrite`-ready) |
| `rgb8` | `cv2.COLOR_RGB2BGR` | `cv2.cvtColor` |
| `yuv422_yuy2` | `cv2.COLOR_YUV2BGR_YUYV` | `cv2.cvtColor` |
| `yuv422` | `cv2.COLOR_YUV2BGR_UYVY` | `cv2.cvtColor` |
| (unknown) | — | Fallback: try `reshape(h, w)`, warn |

Each known encoding reshapes from raw bytes to the appropriate numpy shape before any color conversion. Unknown encodings fall back to the existing behavior (treat as mono8 with a warning).

### 3. IMU text file output

On first IMU message, open `{output_dir}/imu_data.txt` for writing. Write a commented header:

```
# timestamp_ns gx gy gz ax ay az
```

Each subsequent IMU message appends one line:

```
1718234567890123456 0.0012 -0.0034 9.8100 0.0001 -0.0002 0.0000
```

Format details:
- Timestamp: integer nanoseconds (`msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec`)
- Values: 4 decimal places (`:.4f`)
- Order: gyro (gx, gy, gz) then accel (ax, ay, az) — same as the console output
- Separator: single space
- File is flushed after each write (or every N messages) for safety
- File is closed on shutdown

Console printing of IMU data is unchanged.

### 4. Subscription adapters

To handle two different message types from one `--camera-topic` argument, the constructor logic becomes:

```
1. Wait for topic to appear (get_topic_names_and_types, retry up to 5s)
2. If type is "sensor_msgs/msg/CompressedImage":
   → self._on_image = self._on_compressed_image (bound method swap)
   → subscribe to CompressedImage
3. If type is "sensor_msgs/msg/Image":
   → subscribe to Image (existing path, with expanded encoding dispatch)
4. Otherwise: raise RuntimeError
```

### 5. New imports

| Import | Purpose |
|--------|---------|
| `from sensor_msgs.msg import CompressedImage` | Compressed image subscription |

Already available from `sensor_msgs` (no new dependency). `cv2.imdecode` is already available via the existing `cv2` import.

## Files changed

| File | Change |
|------|--------|
| `scripts/validate_topics.py` | Add compressed image auto-detection, encoding dispatch table, IMU text file output |

## Testing

- Run against a node publishing compressed images → verify JPEGs are saved correctly
- Run against a node publishing raw mono8 images → verify no regression
- Run against a node publishing color images (bgr8/yuv422) → verify correct color in output JPEGs
- Verify `imu_data.txt` is created, has correct format, and contains all IMU messages
- Verify Ctrl+C shutdown closes the IMU file cleanly
