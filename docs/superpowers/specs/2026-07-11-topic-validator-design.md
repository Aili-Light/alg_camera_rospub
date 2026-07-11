# Design: ROS Topic Recording and Validation Script

**Date:** 2026-07-11
**Status:** Approved
**Package:** `alg_camera_rospub`

## Motivation

We need a quick way to verify that the camera and IMU publisher nodes are working correctly on the target hardware. Rather than a full test framework, a simple Python script that subscribes to the topics, saves data, and prints statistics.

## Goals

- Subscribe to image topic(s) and IMU topic
- Save each image as JPEG with timestamp in filename
- Calculate and print: frame rate, min/max/mean inter-frame interval, total count
- Same for IMU: rate, interval stats, count
- Print one-line per-message summary during recording
- Configurable: topics, output directory, recording duration
- No external dependencies beyond what's already available (rclpy, cv2, numpy)

## Non-Goals

- Pixel-level correctness checking of image content
- IMU sensor value range validation against ground truth
- Automated pass/fail — this is a human-inspection tool

## Design

A single self-contained Python script: `scripts/validate_topics.py`.

### Command line

```
python3 validate_topics.py [--camera-topic /image] [--imu-topic /imu/data] \
    [--output-dir ./recordings] [--duration 10]
```

### Behavior

1. Create output directory with timestamped subfolder (e.g. `recordings/20260711_143052/`)
2. Subscribe to camera topic (`sensor_msgs/msg/Image`) and IMU topic (`sensor_msgs/msg/Imu`)
3. On each image message:
   - Decode `mono8` to `cv::Mat` equivalent (numpy array via `cv_bridge` or manual)
   - Write JPEG: `image_<seq>_<stamp_ns>.jpg`
   - Record timestamp in a list
   - Print: `[IMG #0001] stamp=123456789.500 size=640x400`
4. On each IMU message:
   - Record timestamp and accel/gyro values in a list
   - Print: `[IMU #0001] stamp=123456789.520 accel=(0.12,-0.03,9.81) gyro=(0.01,-0.02,0.00)`
5. On shutdown (Ctrl+C or timeout):
   - Compute and print summary tables:

```
=== Image Statistics ===
Total frames:      600
Expected rate:      60.0 Hz
Actual rate:        59.8 Hz
Interval min/mean/max: 14.2 / 16.7 / 22.1 ms
First stamp:        123456789.500
Last stamp:         123456799.520
Saved to:           recordings/20260711_143052/

=== IMU Statistics ===
Total samples:      2000
Expected rate:      200.0 Hz
Actual rate:        199.7 Hz
Interval min/mean/max: 4.8 / 5.0 / 5.3 ms
First stamp:        123456789.520
Last stamp:         123456799.515
```

### Dependencies (already available)

- `rclpy` — ROS2 Python client
- `numpy` — image array handling
- `cv2` (opencv-python) — JPEG encoding
- `sensor_msgs` — already in workspace

No `cv_bridge` needed — we decode `mono8` manually from the raw `msg.data` bytes (which is trivial since mono8 is raw 8-bit grayscale).

### Files

| File | Purpose |
|------|---------|
| `scripts/validate_topics.py` | Main recording + validation script |
