# Design: RGB Output and Compressed Image Publishing

**Date:** 2026-07-11
**Status:** Approved
**Package:** `alg_camera_rospub`

## Motivation

1. Currently, `V4L2Capture::grab()` extracts only the Y (luma) channel from YUYV frames, producing mono8 grayscale. The user wants **color RGB** output.
2. The rectified image publisher is unused on non-fisheye setups. Instead, the user wants **compressed JPEG** publishing as an alternative output mode.

## Changes

### 1. Color mode — `V4L2Capture` changes

The `extract_luma_frame()` function currently produces `CV_8UC1` (grayscale). We add a companion function that produces `CV_8UC3` (BGR color) via `cv::cvtColor`:

- For YUYV-family formats: use `cv::cvtColor(yuyv_mat, bgr_output, cv::COLOR_YUV2BGR_YUYV)` (or the appropriate variant for UYVY/VYUY/YVYU).
- For NV12/NV21: use `cv::cvtColor(nv12_mat, bgr_output, cv::COLOR_YUV2BGR_NV12)`.

A new `V4L2Config` field `output_color` (bool, default false) controls whether `grab()` returns mono8 or BGR. The header comment on `grab()` updates accordingly.

No change to `grab()` return type — it still returns `std::optional<cv::Mat>`. The caller inspects the Mat channels to determine color vs grayscale.

### 2. Compressed image publishing — node changes

Replace the rectified image publisher with a compressed image publisher.

- **Remove:** `publish_rectified`, `calib_path` parameters; `rect_pub_`, `rect_maps_` members; `cv::remap` rectification block; `RectificationMaps` dependency from node.
- **Add:** new parameter `publish_compressed` (bool, default false). When true, the node:
  1. Publishes the raw image on the normal topic (as before).
  2. Encodes the frame as JPEG via `cv::imencode(".jpg", frame, params)`.
  3. Publishes a `sensor_msgs/msg/CompressedImage` on `{topic_name}/compressed`.

The compressed message format is already available in `sensor_msgs` (no new ROS dependency). JPEG quality is configurable via a `compressed_quality` parameter (int, default 80, range 1-100).

### 3. What goes away

- `include/alg_camera_rospub/rectification.hpp` — no longer used
- `src/rectification.cpp` — no longer used
- `publish_rectified`, `calib_path` / `stereo_calib_path` parameters from both single and stereo nodes
- The `rect_pub_` member and `cv::remap` block in both nodes
- `RectificationMaps` struct and `load_rectification_maps()` / `validate_rectification_maps()` functions

## New parameter table

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `output_color` | bool | `false` | If true, `grab()` returns color (BGR, 3-channel). If false, mono8 as before. |
| `publish_compressed` | bool | `false` | If true, also publish `CompressedImage` on `{topic_name}/compressed` |
| `compressed_quality` | int | `80` | JPEG quality 1-100 (only used if `publish_compressed=true`) |

## Files changed

| File | Change |
|------|--------|
| `include/alg_camera_rospub/v4l2_capture.hpp` | Add `output_color` to `V4L2Config`, update `grab()` doc |
| `src/v4l2_capture.cpp` | Add `extract_color_frame()` for BGR output; route based on config |
| `src/single_camera_node.cpp` | Replace rect_pub with compressed_pub; add params; JPEG encode |
| `src/stereo_camera_node.cpp` | Same replacement; remove rectification loading |
| `include/alg_camera_rospub/rectification.hpp` | **Delete** |
| `src/rectification.cpp` | **Delete** |
| `CMakeLists.txt` | Remove `src/rectification.cpp` from v4l2_capture sources |
| `launch/single_camera_with_imu.launch.py` | Replace `publish_rectified`/`calib_path` with `publish_compressed`/`compressed_quality` |
| `launch/stereo_camera_with_imu.launch.py` | Same replacement; remove `stereo_calib_path` |
| `README.md` | Update parameter tables and launch examples |

## Validation script update

`scripts/validate_topics.py` should also subscribe to the compressed topic (`/image/compressed`) when `--check-compressed` is passed, and save the decoded JPEGs for visual comparison.

## Dependencies

No new dependencies. `cv::cvtColor` (imgproc) and `cv::imencode` (imgcodecs) are already available via OpenCV, and `sensor_msgs/msg/CompressedImage` is already a dependency.
