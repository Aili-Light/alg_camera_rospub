# Validate Topics — Compressed Image + IMU Text Output Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend `scripts/validate_topics.py` to auto-detect and decode `CompressedImage` topics, handle multiple raw image encodings (mono8, bgr8, rgb8, YUV variants), and save IMU data to a text file.

**Architecture:** All changes are in a single file: `scripts/validate_topics.py`. The script inspects the camera topic type at startup and creates the appropriate subscription (Image or CompressedImage). Image decoding uses a dispatch table keyed by `msg.encoding`. IMU output writes space-delimited lines with nanosecond timestamps to `{output_dir}/imu_data.txt`.

**Tech Stack:** Python 3, rclpy, numpy, cv2 (OpenCV), sensor_msgs

## Global Constraints

- No new CLI arguments — behavior change is transparent to the user
- No new dependencies beyond what's already available (rclpy, cv2, numpy, sensor_msgs)
- Existing image saving behavior (JPEG, timestamped filenames) unchanged
- Console printing of IMU data unchanged
- All changes in `scripts/validate_topics.py`

---

### Task 1: Raw Image Encoding Dispatch Table

**Files:**
- Modify: `scripts/validate_topics.py:61-78` (the `_on_image` method)

**Interfaces:**
- Consumes: `sensor_msgs.msg.Image` messages (existing subscription)
- Produces: `_on_image` handles encodings: `mono8`, `bgr8`, `rgb8`, `yuv422_yuy2`, `yuv422`, plus unknown fallback

- [ ] **Step 1: Replace the mono8-only decode logic with an encoding dispatch table**

Replace lines 68-78 in `_on_image` (the `if msg.encoding == 'mono8':` block through the reshape) with:

```python
        # Decode raw image bytes based on encoding
        encoding = msg.encoding
        if encoding == 'mono8':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width)
        elif encoding == 'bgr8':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
        elif encoding == 'rgb8':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif encoding == 'yuv422_yuy2':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 2)
            arr = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_YUYV)
        elif encoding == 'yuv422':
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 2)
            arr = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_UYVY)
        else:
            # Fallback for unrecognized encodings — try to interpret as mono8
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width)
            self.get_logger().warn(
                f'Unexpected encoding "{encoding}", treating as mono8')
```

- [ ] **Step 2: Verify syntax**

Run: `python3 -m py_compile scripts/validate_topics.py`
Expected: no output (no syntax errors)

- [ ] **Step 3: Commit**

```bash
git add scripts/validate_topics.py
git commit -m "feat: add encoding dispatch table for mono8/bgr8/rgb8/YUV formats

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: CompressedImage Auto-Detection

**Files:**
- Modify: `scripts/validate_topics.py:10-23` (imports), `:26-59` (constructor), add `_on_compressed_image` method

**Interfaces:**
- Consumes: `get_topic_names_and_types()` to inspect topic type; `CompressedImage` message type
- Produces: `_resolve_camera_subscription()` method, `_on_compressed_image()` callback; self.img_sub is either Image or CompressedImage subscription

- [ ] **Step 1: Add CompressedImage import**

Replace line 22:
```python
from sensor_msgs.msg import Image, Imu
```
with:
```python
from sensor_msgs.msg import CompressedImage, Image, Imu
```

- [ ] **Step 2: Add topic type resolution method and compressed callback**

After the `__init__` method (after line 59), before `_on_image`, add the `_resolve_camera_subscription` method. Then after `_on_image` (before `_on_imu`), add `_on_compressed_image`.

Replace the subscription creation block in `__init__` (lines 51-55):
```python
        # Subscriptions
        self.img_sub = self.create_subscription(
            Image, camera_topic, self._on_image, best_effort)
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self._on_imu, reliable)
```
with:
```python
        # Subscriptions
        self._resolve_camera_subscription(camera_topic, best_effort)
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self._on_imu, reliable)
```

Add the `_resolve_camera_subscription` method after `__init__` (after line 59, before existing `_on_image`):

```python
    def _resolve_camera_subscription(self, camera_topic: str, qos: QoSProfile):
        """Inspect topic type and subscribe to Image or CompressedImage."""
        import time as _time
        max_retries = 5
        for attempt in range(max_retries):
            topics_and_types = self.get_topic_names_and_types()
            topic_map = {name: types for name, types in topics_and_types}
            if camera_topic in topic_map:
                type_list = topic_map[camera_topic]
                if 'sensor_msgs/msg/CompressedImage' in type_list:
                    self.get_logger().info(
                        f'Detected compressed image topic: {camera_topic}')
                    self.img_sub = self.create_subscription(
                        CompressedImage, camera_topic,
                        self._on_compressed_image, qos)
                    return
                elif 'sensor_msgs/msg/Image' in type_list:
                    self.get_logger().info(
                        f'Detected raw image topic: {camera_topic}')
                    self.img_sub = self.create_subscription(
                        Image, camera_topic, self._on_image, qos)
                    return
                else:
                    self.get_logger().warn(
                        f'Topic {camera_topic} has unexpected type(s): {type_list}')
            self.get_logger().warn(
                f'Waiting for camera topic "{camera_topic}" '
                f'(attempt {attempt + 1}/{max_retries})...')
            _time.sleep(1.0)
        raise RuntimeError(
            f'Camera topic "{camera_topic}" not found after {max_retries} retries')
```

Add the `_on_compressed_image` method after `_on_image` (after line 88, before `_on_imu`):

```python
    def _on_compressed_image(self, msg: CompressedImage):
        """Decode compressed JPEG image, save, record timestamp."""
        self.img_count += 1
        stamp_ns = msg.header.stamp.nanosec + msg.header.stamp.sec * 1_000_000_000
        stamp_sec = stamp_ns / 1e9
        self.image_timestamps.append(stamp_sec)

        # Decode JPEG bytes to numpy array
        jpeg_bytes = np.frombuffer(msg.data, dtype=np.uint8)
        arr = cv2.imdecode(jpeg_bytes, cv2.IMREAD_COLOR)

        if arr is None:
            self.get_logger().error(
                f'Failed to decode compressed image #{self.img_count}')
            return

        # Save as JPEG (re-encode — preserves original quality)
        filename = f'image_{self.img_count:04d}_{stamp_ns}.jpg'
        filepath = os.path.join(self.output_dir, filename)
        cv2.imwrite(filepath, arr)

        print(f'[IMG #{self.img_count:04d}] stamp={stamp_sec:.3f} '
              f'size={arr.shape[1]}x{arr.shape[0]} compressed')
```

- [ ] **Step 3: Verify syntax**

Run: `python3 -m py_compile scripts/validate_topics.py`
Expected: no output (no syntax errors)

- [ ] **Step 4: Commit**

```bash
git add scripts/validate_topics.py
git commit -m "feat: auto-detect CompressedImage vs Image topic type

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 3: IMU Text File Output

**Files:**
- Modify: `scripts/validate_topics.py:26-38` (constructor — add file handle), `:89-105` (`_on_imu` — append to file), `:107-120` (`print_summary` — close file)

**Interfaces:**
- Consumes: `msg.header.stamp.sec`, `msg.header.stamp.nanosec`, `msg.linear_acceleration.x/y/z`, `msg.angular_velocity.x/y/z`
- Produces: `{output_dir}/imu_data.txt` with header line and space-delimited data rows

- [ ] **Step 1: Open IMU file on first write**

In `__init__`, add a file handle after the existing state variables (after line 38):

```python
        # IMU text file output (opened on first message)
        self.imu_file = None
```

- [ ] **Step 2: Append IMU data to text file**

Replace the `_on_imu` method (lines 89-105) with:

```python
    def _on_imu(self, msg: Imu):
        """Record IMU sample, print summary, and append to text file."""
        self.imu_count += 1
        stamp_ns = msg.header.stamp.nanosec + msg.header.stamp.sec * 1_000_000_000
        stamp_sec = stamp_ns / 1e9
        self.imu_timestamps.append(stamp_sec)

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # Open IMU text file on first message
        if self.imu_file is None:
            filepath = os.path.join(self.output_dir, 'imu_data.txt')
            self.imu_file = open(filepath, 'w')
            self.imu_file.write(
                '# timestamp_ns gx gy gz ax ay az\n')

        # Append one space-delimited line: ts_ns gx gy gz ax ay az
        self.imu_file.write(
            f'{stamp_ns} {gx:.4f} {gy:.4f} {gz:.4f} '
            f'{ax:.4f} {ay:.4f} {az:.4f}\n')
        self.imu_file.flush()

        print(f'[IMU #{self.imu_count:04d}] stamp={stamp_sec:.3f} '
              f'accel=({ax:.2f},{ay:.2f},{az:.2f}) '
              f'gyro=({gx:.2f},{gy:.2f},{gz:.2f})')
```

- [ ] **Step 3: Close IMU file in print_summary**

In `print_summary` (after line 111), add file close logic before the image statistics block. Replace lines 111-112:

```python
        print()
        print('=' * 60)
```

with:

```python
        # Close IMU text file if open
        if self.imu_file is not None:
            self.imu_file.close()
            self.imu_file = None
            imu_path = os.path.join(self.output_dir, 'imu_data.txt')
            print(f'\nIMU data saved to: {imu_path}')

        print()
        print('=' * 60)
```

- [ ] **Step 4: Verify syntax**

Run: `python3 -m py_compile scripts/validate_topics.py`
Expected: no output (no syntax errors)

- [ ] **Step 5: Commit**

```bash
git add scripts/validate_topics.py
git commit -m "feat: save IMU data to space-delimited text file with nanosecond timestamps

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 4: Integration Smoke Test

**Files:**
- No file changes — verification only

**Interfaces:**
- Consumes: the fully modified `scripts/validate_topics.py`
- Produces: confirmation that the script imports and loads without errors

- [ ] **Step 1: Verify the full script loads and parses help text**

Run: `python3 scripts/validate_topics.py --help`
Expected: argparse help text printed, no import errors, clean exit code 0

- [ ] **Step 2: Verify by reading the final script for consistency**

Check that:
- `CompressedImage` is imported
- `_resolve_camera_subscription` is called from `__init__` 
- `_on_compressed_image` exists and uses `cv2.imdecode`
- `_on_image` has the full encoding dispatch table
- `_on_imu` writes to `self.imu_file` and flushes
- `print_summary` closes `self.imu_file`
- `self.imu_file` is initialized to `None` in `__init__`

Run: `python3 -m py_compile scripts/validate_topics.py`
Expected: no errors

- [ ] **Step 3: Commit any final fixes**

If the review found issues, fix them and commit:

```bash
git add scripts/validate_topics.py
git commit -m "chore: final polish on validate_topics compressed + IMU support

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```
