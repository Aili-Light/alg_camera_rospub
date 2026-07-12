# `--check-only` Timestamp Validation Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `--check-only` mode to `scripts/decode_rosbag.py` that skips file I/O and prints a timestamp consistency report with intra-topic interval statistics (1%/5%/10% distribution) and cross-topic image-to-IMU offset analysis.

**Architecture:** The existing `decode_rosbag.py` gets a new `--check-only` flag. When set, the read loop collects nanosecond timestamps only (no decode, no I/O), then two new report functions compute and print the interval consistency report. The normal decode-and-save path is untouched. All computation uses numpy (already imported).

**Tech Stack:** Python 3, numpy, rosbag2_py

## Global Constraints

- No new dependencies — all computation uses numpy (already available)
- No new files — all changes in `scripts/decode_rosbag.py`
- Normal mode (without `--check-only`) must be unchanged — full regression
- Report format must match the design spec exactly

---

### Task 1: Add `--check-only` Mode to decode_rosbag.py

**Files:**
- Modify: `scripts/decode_rosbag.py`

**Interfaces:**
- Consumes: existing `main()`, `_print_stream_stats()`, `rosbag2_py` reader
- Produces: `--check-only` CLI arg, check-only read loop, `_print_interval_report()`, `_print_sync_report()`, `_print_check_report()`

- [ ] **Step 1: Add the `--check-only` CLI argument**

In `main()`, after the existing `--imu-topic` argument (after line 87), add:

```python
    parser.add_argument(
        '--check-only', action='store_true', default=False,
        help='Validate timestamps only; do not save data')
```

- [ ] **Step 2: Add the interval consistency report function**

Add before `main()` (before line 70), after `_print_summary`:

```python
def _print_interval_report(name, topic_name, timestamps_ns):
    """Print intra-topic interval consistency for one stream."""
    count = len(timestamps_ns)
    if count < 2:
        print(f'\n{name} ({topic_name}):')
        print(f'  Count:              {count}')
        print(f'  Not enough data for interval analysis (need >= 2).')
        return

    intervals_ms = np.diff(timestamps_ns) / 1e6  # ns -> ms
    mean_interval_ms = intervals_ms.mean()
    expected_hz = 1000.0 / mean_interval_ms if mean_interval_ms > 0 else 0.0
    median_interval_ms = np.median(intervals_ms)
    std_interval_ms = intervals_ms.std()

    print(f'\n{name} ({topic_name}):')
    print(f'  Count:              {count}')
    print(f'  Mean interval:      {mean_interval_ms:.2f} ms  '
          f'(expected {expected_hz:.0f} Hz)')
    print(f'  Interval stats:')
    print(f'    min/mean/max:     {intervals_ms.min():.1f} / '
          f'{mean_interval_ms:.1f} / {intervals_ms.max():.1f} ms')
    print(f'    median/std:       {median_interval_ms:.1f} / '
          f'{std_interval_ms:.1f} ms')

    # Distribution: % within 1%, 5%, 10% of mean
    for pct in [1, 5, 10]:
        threshold = pct / 100.0
        within = np.sum(
            np.abs(intervals_ms - mean_interval_ms) / mean_interval_ms
            <= threshold)
        pct_total = within / len(intervals_ms) * 100.0
        print(f'  Within ±{pct:2d}% of mean:  '
              f'{pct_total:5.1f}%  ({within} / {len(intervals_ms)})')
```

- [ ] **Step 3: Add the cross-topic synchronization report function**

Add after `_print_interval_report`, before `main()`:

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
```

- [ ] **Step 4: Add the top-level check-only report orchestrator**

Add after `_print_sync_report`, before `main()`:

```python
def _print_check_report(image_timestamps_ns, imu_timestamps_ns,
                        camera_topic, imu_topic):
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

    print()
    print('=' * 60)
```

- [ ] **Step 5: Add the check-only branch to the main read loop**

Replace lines 108-195 (the state variables through end of read loop) — or more precisely, restructure `main()` to branch on `args.check_only` after the topic-type-map. Insert after line 120 (`print(f'Bag contains topics: {topic_type_map}')`):

```python
    if args.check_only:
        # --- Check-only mode: collect timestamps, print report ---
        image_timestamps_ns = []
        imu_timestamps_ns = []

        print('Reading messages (check-only mode)...')
        while reader.has_next():
            (topic_name, raw_msg, stamp_ns) = reader.read_next()

            if topic_name == args.camera_topic:
                image_timestamps_ns.append(stamp_ns)
                img_count += 1
                if img_count % 100 == 0:
                    print(f'  ... {img_count} images, '
                          f'{imu_count} IMU samples')

            elif topic_name == args.imu_topic:
                imu_timestamps_ns.append(stamp_ns)
                imu_count += 1
                if imu_count % 2000 == 0:
                    print(f'  ... {img_count} images, '
                          f'{imu_count} IMU samples')

        print(f'Done. {img_count} images, {imu_count} IMU samples.')

        _print_check_report(image_timestamps_ns, imu_timestamps_ns,
                           args.camera_topic, args.imu_topic)
        return 0

    # --- Normal mode: decode and save (existing code) ---
    # State
    image_timestamps = []
    imu_timestamps = []
    img_count = 0
    imu_count = 0
    imu_file = None
```

**IMPORTANT:** The existing read loop (lines 122-192) must be preserved exactly below this as the normal-mode path. The new check-only block is inserted *before* the existing read loop, which becomes the `else` branch implicitly via the `return 0` in the check-only block.

- [ ] **Step 6: Verify syntax**

Run: `python3 -m py_compile scripts/decode_rosbag.py`
Expected: no output (no errors)

- [ ] **Step 7: Run help to verify new argument**

Run: `python3 scripts/decode_rosbag.py --help`
Expected: `--check-only` appears in help output

- [ ] **Step 8: Commit**

```bash
git add scripts/decode_rosbag.py
git commit -m "feat: add --check-only timestamp validation mode

- When --check-only is set, skip all file I/O (no JPEGs, no imu_data.txt)
- Collect nanosecond timestamps in memory from both topics
- Print intra-topic interval stats (min/mean/max/median/std)
- Print interval distribution (% within 1%, 5%, 10% of mean)
- Print cross-topic image-to-nearest-IMU offset stats
- Normal decode-and-save mode unchanged

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

### Task 2: End-to-End Test

**Files:**
- No file changes — verification only

- [ ] **Step 1: Run `--check-only` against the real rosbag**

Run: `python3 scripts/decode_rosbag.py --bag-dir recordings/rosbag2_2026_07_12-04_25_53 --check-only`
Expected: report prints with image count 1585, IMU count 10547, interval stats for both streams, distribution percentages, cross-topic offset stats. No files created.

- [ ] **Step 2: Verify no files were created**

Run: `ls -lt recordings/ | head -5`
Expected: no new timestamped subdirectory created (the bag dir is the only recent entry)

- [ ] **Step 3: Run regression — normal mode still works**

Run:
```bash
python3 scripts/decode_rosbag.py --bag-dir recordings/rosbag2_2026_07_12-04_25_53 --output-dir ./recordings --duration-check
# Let it run, then:
OUT_DIR=$(ls -td recordings/2* | head -1)
echo "Images: $(ls "$OUT_DIR"/*.jpg 2>/dev/null | wc -l)"
echo "IMU lines: $(wc -l < "$OUT_DIR/imu_data.txt")"
rm -rf "$OUT_DIR"
```
Expected: 1585 JPEGs, 10548 IMU lines (1 header + 10547 data)

- [ ] **Step 4: Commit (if any fixes needed)**

```bash
git add scripts/decode_rosbag.py
git commit -m "chore: end-to-end test fixes for --check-only mode

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```
