# Histogram + Timestamp Dump Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add ASCII histogram of image-to-IMU offset distribution and save raw timestamp text files in `--check-only` mode.

**Architecture:** Three focused edits to `scripts/decode_rosbag.py`: (1) append histogram rendering to `_print_sync_report()` using `np.histogram`, (2) add `_write_timestamp_files()` function, (3) call it from the check-only path in `main()` with an `output_dir` parameter.

**Tech Stack:** Python 3, numpy

## Global Constraints

- All changes in `scripts/decode_rosbag.py`
- No new dependencies
- Histogram: 20 bins, ASCII `#` bars, 60-char max width
- Timestamp files: one nanosecond integer per line, no header
- All outputs only in `--check-only` mode
- Normal mode must be unchanged

---

### Task 1: Add Histogram + Timestamp Dump

**Files:**
- Modify: `scripts/decode_rosbag.py`

**Interfaces:**
- Consumes: `_print_sync_report()`, `_print_check_report()`, check-only block in `main()`
- Produces: histogram output in `_print_sync_report()`, `_write_timestamp_files()`, `output_dir` param on `_print_check_report()`

- [ ] **Step 1: Add histogram section to `_print_sync_report()`**

Replace the entire `_print_sync_report` function (lines 106-135) with:

```python
def _print_sync_report(image_timestamps_ns, imu_timestamps_ns):
    """Print cross-topic image-to-nearest-IMU offset statistics."""
    if len(image_timestamps_ns) == 0:
        print('\nNo image data for synchronization analysis.')
        return
    if len(imu_timestamps_ns) == 0:
        print('\nNo IMU data for synchronization analysis.')
        return

    imu_array = np.array(imu_timestamps_ns)
    offsets_ms = []
    for t_img in image_timestamps_ns:
        idx = np.searchsorted(imu_array, t_img)
        # Check nearest neighbor (idx-1 and idx, clamped to bounds)
        candidates = []
        if idx > 0:
            candidates.append(abs(t_img - imu_array[idx - 1]))
        if idx < len(imu_array):
            candidates.append(abs(t_img - imu_array[idx]))
        offset_ns = min(candidates)
        offsets_ms.append(offset_ns / 1e6)

    offsets = np.array(offsets_ms)
    print(f'\n=== Cross-Topic Synchronization ===')
    print(f'\nImage-to-nearest-IMU offset:')
    print(f'  Count:              {len(offsets)}')
    print(f'  min/mean/max:       {offsets.min():.1f} / '
          f'{offsets.mean():.1f} / {offsets.max():.1f} ms')
    print(f'  median/std:         {np.median(offsets):.1f} / '
          f'{offsets.std():.1f} ms')

    # ASCII histogram (20 bins)
    counts, bin_edges = np.histogram(offsets, bins=20)
    max_count = counts.max()
    bar_width = 60 if max_count > 0 else 1

    print(f'\nOffset histogram (ms):')
    for i in range(len(counts)):
        if counts[i] == 0:
            continue
        bar_len = int(counts[i] / max_count * bar_width)
        pct = counts[i] / len(offsets) * 100.0
        bar = '#' * bar_len
        print(f'  {bin_edges[i]:5.1f} - '
              f'{bin_edges[i+1]:5.1f} | {bar:<{bar_width}} '
              f'{counts[i]:>5} ({pct:5.1f}%)')
```

- [ ] **Step 2: Add `_write_timestamp_files()` function**

Add after `_print_sync_report`, before `_print_check_report`:

```python
def _write_timestamp_files(image_timestamps_ns, imu_timestamps_ns,
                           output_dir):
    """Save raw timestamps to text files (one ns integer per line, no header)."""
    filepath = os.path.join(output_dir, 'image_timestamps.txt')
    with open(filepath, 'w') as f:
        for ts in image_timestamps_ns:
            f.write(f'{ts}\n')
    print(f'Image timestamps saved to: {filepath}')

    filepath = os.path.join(output_dir, 'imu_timestamps.txt')
    with open(filepath, 'w') as f:
        for ts in imu_timestamps_ns:
            f.write(f'{ts}\n')
    print(f'IMU timestamps saved to: {filepath}')
```

- [ ] **Step 3: Update `_print_check_report()` to accept `output_dir` and call writer**

Replace `_print_check_report` (lines 138-154) with:

```python
def _print_check_report(image_timestamps_ns, imu_timestamps_ns,
                        camera_topic, imu_topic, output_dir):
    """Print the full timestamp consistency report."""
    print()
    print('=' * 60)
    print('=== Timestamp Consistency Report ===')

    print('\n=== Intra-Topic Interval Consistency ===')
    _print_interval_report('Image stream', camera_topic,
                           image_timestamps_ns)
    _print_interval_report('IMU stream', imu_topic,
                           imu_timestamps_ns)

    _print_sync_report(image_timestamps_ns, imu_timestamps_ns)

    if output_dir is not None:
        print()
        _write_timestamp_files(image_timestamps_ns, imu_timestamps_ns,
                               output_dir)

    print()
    print('=' * 60)
```

- [ ] **Step 4: Update check-only block in `main()` to create output dir and pass to report**

Replace lines 226-229 (the `_print_check_report` call in the check-only block) with:

```python
        # Create timestamped output directory for timestamp files
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        check_output_dir = os.path.join(args.output_dir, timestamp_str)
        os.makedirs(check_output_dir, exist_ok=True)

        _print_check_report(image_timestamps_ns, imu_timestamps_ns,
                           args.camera_topic, args.imu_topic,
                           check_output_dir)
```

- [ ] **Step 5: Verify syntax**

Run: `python3 -m py_compile scripts/decode_rosbag.py`
Expected: no output (no errors)

- [ ] **Step 6: Run help**

Run: `python3 scripts/decode_rosbag.py --help`
Expected: all arguments listed, exit 0

- [ ] **Step 7: Commit**

```bash
git add scripts/decode_rosbag.py
git commit -m "feat: add offset histogram and timestamp dump to --check-only

- ASCII histogram of image-to-IMU offset distribution (20 bins, np.histogram)
- Save image_timestamps.txt and imu_timestamps.txt in --check-only mode
- Each file: one nanosecond integer per line, no header
- Output to timestamped subfolder under --output-dir

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: End-to-End Test

**Files:**
- No file changes — verification only

- [ ] **Step 1: Run `--check-only` against the real rosbag**

Run: `python3 scripts/decode_rosbag.py --bag-dir recordings/rosbag2_2026_07_12-04_25_53 --check-only 2>&1 | tail -40`
Expected: histogram visible with 20 bins, offset stats, timestamp files created

- [ ] **Step 2: Verify timestamp files**

Run:
```bash
OUT_DIR=$(ls -td recordings/2* | head -1)
echo "Image ts lines: $(wc -l < "$OUT_DIR/image_timestamps.txt")"
echo "IMU ts lines: $(wc -l < "$OUT_DIR/imu_timestamps.txt")"
head -3 "$OUT_DIR/image_timestamps.txt"
head -3 "$OUT_DIR/imu_timestamps.txt"
```
Expected: 1585 lines, 10547 lines. Each line is a nanosecond integer. No headers.

- [ ] **Step 3: Regression — normal mode still works**

Run:
```bash
python3 scripts/decode_rosbag.py --bag-dir recordings/rosbag2_2026_07_12-04_25_53 --output-dir ./recordings 2>&1 | tail -5
OUT_DIR=$(ls -td recordings/2* | head -1)
echo "Images: $(ls "$OUT_DIR"/*.jpg 2>/dev/null | wc -l)"
echo "IMU lines: $(wc -l < "$OUT_DIR/imu_data.txt")"
rm -rf "$OUT_DIR"
```
Expected: 1585 JPEGs, 10548 IMU lines

- [ ] **Step 4: Clean up test output**

```bash
for d in $(ls -td recordings/2*); do rm -rf "$d"; done
```

- [ ] **Step 5: Commit (if any fixes needed)**

```bash
git add scripts/decode_rosbag.py
git commit -m "chore: end-to-end test fixes for histogram + timestamp dump

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```
