# Rosbag Decode Script Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create `scripts/decode_rosbag.py` — a single script that reads a rosbag directory, decodes compressed images to JPEG and saves IMU data to a space-delimited text file, matching the output format of `scripts/validate_topics.py`.

**Architecture:** Pull-based bag reader using `rosbag2_py.SequentialReader`. No ROS2 node or subscriptions — just open the bag, iterate messages, decode by type, save outputs, print statistics. All decoding logic mirrors the existing patterns in `validate_topics.py`.

**Tech Stack:** Python 3, rosbag2_py, numpy, cv2 (OpenCV), sensor_msgs

## Global Constraints

- Output format must match `validate_topics.py` exactly: image filenames `image_{seq:04d}_{stamp_ns}.jpg`, IMU file header `# timestamp_ns gx gy gz ax ay az`, space-delimited data with 4 decimal places
- Timestamped output subfolder: `{output_dir}/YYYYMMDD_HHMMSS/`
- Configurable: `--bag-dir`, `--output-dir`, `--camera-topic`, `--imu-topic`
- No new dependencies beyond what's already available
- All changes in a single new file: `scripts/decode_rosbag.py`

---

### Task 1: Create decode_rosbag.py Script

**Files:**
- Create: `scripts/decode_rosbag.py`

**Interfaces:**
- Produces: `main()` entry point, CLI via argparse (--bag-dir, --output-dir, --camera-topic, --imu-topic)
- Uses: `rosbag2_py.SequentialReader`, `rosbag2_py.StorageOptions`, `rosbag2_py.ConverterOptions`

- [ ] **Step 1: Create the script with shebang, imports, and CLI**

```python
#!/usr/bin/env python3
"""
Decode a ROS2 rosbag: extract compressed images as JPEG and IMU data as text.

Usage:
    python3 decode_rosbag.py --bag-dir ./recordings/rosbag2_2026_07_12-04_25_53 \
        --output-dir ./recordings --camera-topic /image/compressed \
        --imu-topic /imu/data
"""

import argparse
import os
import sys
from datetime import datetime

import cv2
import numpy as np
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, Imu


def _stamp_to_ns(msg):
    """Extract nanosecond timestamp from message header."""
    return msg.header.stamp.nanosec + msg.header.stamp.sec * 1_000_000_000


def _print_stream_stats(timestamps, count, label):
    """Print statistics for a single stream (same format as validate_topics.py)."""
    if count == 0:
        print(f'No {label} data received.')
        return

    total_time = timestamps[-1] - timestamps[0]
    actual_rate = (count - 1) / total_time if total_time > 0 else 0.0

    intervals = np.diff(timestamps) * 1000.0  # convert to ms

    print(f'Total frames:      {count}')
    print(f'Actual rate:       {actual_rate:.1f} Hz')
    print(f'Interval min/mean/max: '
          f'{intervals.min():.1f} / {intervals.mean():.1f} / '
          f'{intervals.max():.1f} ms')
    print(f'First stamp:       {timestamps[0]:.3f}')
    print(f'Last stamp:        {timestamps[-1]:.3f}')
    print(f'Total duration:    {total_time:.1f} s')


def print_summary(image_timestamps, img_count, imu_timestamps, imu_count,
                  imu_file, output_dir):
    """Print statistics and close IMU file."""
    # Close IMU text file if open
    if imu_file is not None:
        imu_file.close()
        imu_path = os.path.join(output_dir, 'imu_data.txt')
        print(f'\nIMU data saved to: {imu_path}')

    print()
    print('=' * 60)
    print('=== Image Statistics ===')
    _print_stream_stats(image_timestamps, img_count, 'image')

    print()
    print('=== IMU Statistics ===')
    _print_stream_stats(imu_timestamps, imu_count, 'IMU')

    print(f'\nSaved images to: {output_dir}')
    print('=' * 60)


def main():
    parser = argparse.ArgumentParser(
        description='Decode a ROS2 rosbag: extract images and IMU data')
    parser.add_argument(
        '--bag-dir', default='./recordings/rosbag2_2026_07_12-04_25_53',
        help='Path to rosbag directory (default: ./recordings/rosbag2_2026_07_12-04_25_53)')
    parser.add_argument(
        '--output-dir', default='./recordings',
        help='Parent output directory — timestamped subfolder created inside '
             '(default: ./recordings)')
    parser.add_argument(
        '--camera-topic', default='/image/compressed',
        help='Compressed image topic name in the bag '
             '(default: /image/compressed)')
    parser.add_argument(
        '--imu-topic', default='/imu/data',
        help='IMU topic name in the bag (default: /imu/data)')
    args = parser.parse_args()

    # Validate bag directory
    if not os.path.isdir(args.bag_dir):
        print(f'Error: bag directory not found: {args.bag_dir}', file=sys.stderr)
        return 1

    # Create timestamped output directory
    timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = os.path.join(args.output_dir, timestamp_str)
    os.makedirs(output_dir, exist_ok=True)
    print(f'Decoding to: {output_dir}')

    # Open the rosbag
    storage_options = StorageOptions(uri=args.bag_dir, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # State
    image_timestamps = []
    imu_timestamps = []
    img_count = 0
    imu_count = 0
    imu_file = None

    # Topic name → type map from bag metadata
    topic_type_map = {}
    for topic_meta in reader.get_all_topics_and_types():
        topic_type_map[topic_meta.name] = topic_meta.type

    print(f'Bag contains topics: {topic_type_map}')

    # Read loop
    while reader.has_next():
        (topic_name, raw_msg, stamp_ns) = reader.read_next()

        if topic_name == args.camera_topic:
            msg_type = topic_type_map.get(topic_name, '')
            if 'CompressedImage' not in msg_type:
                print(f'Warning: unexpected type {msg_type} '
                      f'on topic {topic_name}, skipping', file=sys.stderr)
                continue

            msg = deserialize_message(raw_msg, CompressedImage)
            img_count += 1
            stamp_sec = stamp_ns / 1e9
            image_timestamps.append(stamp_sec)

            # Decode JPEG bytes
            jpeg_bytes = np.frombuffer(msg.data, dtype=np.uint8)
            arr = cv2.imdecode(jpeg_bytes, cv2.IMREAD_COLOR)

            if arr is None:
                print(f'[IMG #{img_count:04d}] ERROR: failed to decode '
                      f'compressed image', file=sys.stderr)
                continue

            # Save as JPEG
            filename = f'image_{img_count:04d}_{stamp_ns}.jpg'
            filepath = os.path.join(output_dir, filename)
            cv2.imwrite(filepath, arr)

            print(f'[IMG #{img_count:04d}] stamp={stamp_sec:.3f} '
                  f'size={arr.shape[1]}x{arr.shape[0]} compressed')

        elif topic_name == args.imu_topic:
            msg = deserialize_message(raw_msg, Imu)
            imu_count += 1
            stamp_sec = stamp_ns / 1e9
            imu_timestamps.append(stamp_sec)

            ax = msg.linear_acceleration.x
            ay = msg.linear_acceleration.y
            az = msg.linear_acceleration.z
            gx = msg.angular_velocity.x
            gy = msg.angular_velocity.y
            gz = msg.angular_velocity.z

            # Open IMU text file on first message
            if imu_file is None:
                filepath = os.path.join(output_dir, 'imu_data.txt')
                imu_file = open(filepath, 'w')
                imu_file.write('# timestamp_ns gx gy gz ax ay az\n')

            # Append one space-delimited line
            imu_file.write(
                f'{stamp_ns} {gx:.4f} {gy:.4f} {gz:.4f} '
                f'{ax:.4f} {ay:.4f} {az:.4f}\n')
            imu_file.flush()

            print(f'[IMU #{imu_count:04d}] stamp={stamp_sec:.3f} '
                  f'accel=({ax:.2f},{ay:.2f},{az:.2f}) '
                  f'gyro=({gx:.2f},{gy:.2f},{gz:.2f})')

        else:
            print(f'Warning: skipping unknown topic "{topic_name}"', file=sys.stderr)

    # Print summary
    print_summary(image_timestamps, img_count, imu_timestamps, imu_count,
                  imu_file, output_dir)
    return 0


if __name__ == '__main__':
    sys.exit(main())
```

- [ ] **Step 2: Verify syntax**

Run: `python3 -m py_compile scripts/decode_rosbag.py`
Expected: no output (no syntax errors)

- [ ] **Step 3: Verify imports load**

Run: `python3 -c 'import ast; ast.parse(open("scripts/decode_rosbag.py").read()); print("AST OK")'`
Expected: `AST OK`

- [ ] **Step 4: Run help text check**

Run: `python3 scripts/decode_rosbag.py --help`
Expected: argparse help text with all four flags printed, exit code 0

- [ ] **Step 5: Commit**

```bash
git add scripts/decode_rosbag.py
git commit -m "feat: add rosbag decode script for compressed images and IMU data

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: End-to-End Test with Real Rosbag

**Files:**
- No file changes — verification only

**Interfaces:**
- Consumes: `scripts/decode_rosbag.py` + `recordings/rosbag2_2026_07_12-04_25_53/`
- Produces: JPEG images + IMU text file in timestamped output directory

- [ ] **Step 1: Run the script against the real rosbag**

Run: `python3 scripts/decode_rosbag.py --bag-dir recordings/rosbag2_2026_07_12-04_25_53 --output-dir ./recordings --camera-topic /image/compressed --imu-topic /imu/data`
Expected: script runs to completion; 1585 image lines printed; 10547 IMU lines printed; statistics summary at end

- [ ] **Step 2: Verify output counts**

Run:
```bash
OUT_DIR=$(ls -td recordings/2* | head -1)
echo "Output: $OUT_DIR"
echo "Image count: $(ls "$OUT_DIR"/*.jpg 2>/dev/null | wc -l)"
echo "IMU lines (incl header): $(wc -l < "$OUT_DIR/imu_data.txt")"
head -3 "$OUT_DIR/imu_data.txt"
```
Expected: 1585 JPEGs, 10548 lines (1 header + 10547 data), IMU file starts with `# timestamp_ns gx gy gz ax ay az` header

- [ ] **Step 3: Verify statistics match rosbag metadata**

Run: `ros2 bag info recordings/rosbag2_2026_07_12-04_25_53/`
Expected: message counts match (CompressedImage: 1585, Imu: 10547)

- [ ] **Step 4: Spot-check one IMU line**

Run:
```bash
OUT_DIR=$(ls -td recordings/2* | head -1)
sed -n '2p' "$OUT_DIR/imu_data.txt" | awk '{print NF, $1}'
```
Expected: `7` fields, timestamp is a large integer (nanoseconds since epoch)

- [ ] **Step 5: Clean up test outputs**

```bash
OUT_DIR=$(ls -td recordings/2* | head -1)
rm -rf "$OUT_DIR"
```

- [ ] **Step 6: Commit (if any fixes were needed)**

```bash
git add scripts/decode_rosbag.py
git commit -m "chore: end-to-end test fixes for decode_rosbag.py

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```
