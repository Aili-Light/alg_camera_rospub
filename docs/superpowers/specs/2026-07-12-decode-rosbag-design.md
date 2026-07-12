# Design: Rosbag Decode Script

**Date:** 2026-07-12
**Status:** Draft
**Package:** `alg_camera_rospub`

## Motivation

The user has recorded rosbag data containing `/image/compressed` and `/imu/data` topics. They need a script to decode the rosbag offline and produce the same outputs as `scripts/validate_topics.py`: JPEG images and a space-delimited IMU text file with nanosecond timestamps.

## Goals

- Read a rosbag directory using `rosbag2_py.SequentialReader`
- Decode `sensor_msgs/msg/CompressedImage` messages via `cv2.imdecode` and save as JPEG
- Save `sensor_msgs/msg/Imu` messages to a space-delimited text file with nanosecond timestamps
- Print per-message status lines during decoding
- Print statistics summary at completion (rate, interval stats, count, duration)
- Output format matches `validate_topics.py` exactly (filenames, IMU file format, directory structure)
- Configurable: bag directory, output directory, camera topic name, IMU topic name

## Non-Goals

- Live topic subscription (use `validate_topics.py` for that)
- Support for raw `Image` topics in the bag (can be added later if needed)
- Bag recording or manipulation — decode only

## Design

A single self-contained Python script: `scripts/decode_rosbag.py`.

### Command line

```
python3 decode_rosbag.py [--bag-dir ./recordings/rosbag2_2026_07_12-04_25_53] \
    [--output-dir ./recordings] [--camera-topic /image/compressed] \
    [--imu-topic /imu/data]
```

| Argument | Default | Description |
|----------|---------|-------------|
| `--bag-dir` | `./recordings/latest` | Path to rosbag directory (containing metadata.yaml + .db3) |
| `--output-dir` | `./recordings` | Parent output directory (timestamped subfolder created inside) |
| `--camera-topic` | `/image/compressed` | Compressed image topic name in the bag |
| `--imu-topic` | `/imu/data` | IMU topic name in the bag |

### Behavior

1. Create timestamped output subfolder: `{output_dir}/YYYYMMDD_HHMMSS/`
2. Open the rosbag with `rosbag2_py.SequentialReader` (configured for `cdr` serialization)
3. Open the IMU text file on first IMU message: write `# timestamp_ns gx gy gz ax ay az` header
4. Iterate through all messages:
   - **CompressedImage**: decode JPEG bytes via `cv2.imdecode`, save as `image_{seq:04d}_{stamp_ns}.jpg`, print status
   - **Imu**: append one space-delimited line to `imu_data.txt`, flush, print status
   - **Other**: skip with a warning
5. At end, print the same statistics summary as `validate_topics.py` (rate, interval min/mean/max, count, duration)
6. Close the IMU file

### Output format (identical to `validate_topics.py`)

**Image filenames:** `image_0001_<stamp_ns>.jpg`

**IMU text file:** `imu_data.txt`
```
# timestamp_ns gx gy gz ax ay az
1783830353844253837 0.0012 -0.0034 9.8100 0.0001 -0.0002 0.0000
...
```

**Console status lines:**
```
[IMG #0001] stamp=1783830353.844 size=640x400 compressed
[IMU #0001] stamp=1783830353.845 accel=(0.12,-0.03,9.81) gyro=(0.01,-0.02,0.00)
```

**Statistics summary:**
```
=== Image Statistics ===
Total frames:      1585
Actual rate:        30.0 Hz
Interval min/mean/max: 32.1 / 33.3 / 35.2 ms
First stamp:        1783830353.844
Last stamp:         1783830406.655
Total duration:     52.8 s

=== IMU Statistics ===
Total samples:      10547
Actual rate:        199.7 Hz
Interval min/mean/max: 4.8 / 5.0 / 5.3 ms
First stamp:        1783830353.844
Last stamp:         1783830406.655
Total duration:     52.8 s

Saved images to: ./recordings/20260712_120000/
IMU data saved to: ./recordings/20260712_120000/imu_data.txt
```

### Architecture

The script uses a pull-based bag reader (no ROS2 node, no subscriptions):

```
rosbag2_py.SequentialReader
    → read next message
    → if CompressedImage: decode JPEG → save file
    → if Imu: format line → append to file
    → loop until bag exhausted
    → print statistics
```

This is simpler than `validate_topics.py` — no `rclpy.init`, no `spin_once`, no signal handlers needed.

### Dependencies

| Package | Purpose | Availability |
|---------|---------|-------------|
| `rosbag2_py` | Read rosbag2 files | Part of ROS2 Humble+ install |
| `rclpy` | Deserialize serialized messages (via `deserialize_message`) | Already in workspace |
| `cv2` (opencv-python) | JPEG decode + encode | Already available |
| `numpy` | Array handling | Already available |
| `sensor_msgs` | CompressedImage, Imu message types | Already in workspace |

### Error handling

- Bag directory not found → print error and exit
- `metadata.yaml` missing or corrupt → let `rosbag2_py` raise; print friendly error
- Corrupt JPEG in compressed message → log error, skip that frame, continue
- Disk full during output → let Python raise; operator fixes space and re-runs

### Files

| File | Purpose |
|------|---------|
| `scripts/decode_rosbag.py` | New rosbag decode script |

### Testing

- Run against `recordings/rosbag2_2026_07_12-04_25_53/` → verify 1585 images saved, `imu_data.txt` has 10547 data lines + 1 header
- Compare a few output lines against known values from the bag for correctness
- Verify statistics match `ros2 bag info` output (message counts)
