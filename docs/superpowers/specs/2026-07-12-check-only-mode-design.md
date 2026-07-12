# Design: `--check-only` Timestamp Validation Mode for decode_rosbag.py

**Date:** 2026-07-12
**Status:** Draft
**Package:** `alg_camera_rospub`

## Motivation

The user wants to validate rosbag timestamp consistency without saving decoded data. This covers two checks:

1. **Intra-topic**: Are consecutive messages within each topic uniformly spaced? How much jitter is present?
2. **Cross-topic**: What is the image-to-nearest-IMU offset, and is it consistent across frames?

## Goals

- Add `--check-only` flag to `scripts/decode_rosbag.py`
- In check-only mode: skip all file I/O (no JPEGs, no `imu_data.txt`, no output directory)
- Collect nanosecond timestamps for both topics in memory
- Compute and print a timestamp consistency report:
  - Per-topic interval statistics (min/max/mean/median/std)
  - Per-topic interval distribution (% within ±1%, ±5%, ±10% of mean)
  - Cross-topic image-to-nearest-IMU offset statistics (min/max/mean/median/std)
- No new dependencies — all computation uses numpy (already available)

## Non-Goals

- Pass/fail verdicts with configurable thresholds (statistics only, human judges)
- Timestamp ordering validation (assumes messages are temporally ordered in the bag)
- Wall-clock vs. system-clock comparison

## Design

All changes are in `scripts/decode_rosbag.py`.

### CLI

Add one new argument:

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `--check-only` | `store_true` | `False` | Validate timestamps only; do not save data |

When `--check-only` is set, `--output-dir` is ignored (no directory needed, no files written).

### Behavior

**Normal mode** (unchanged): Decode images to JPEG, save IMU to text file, print statistics.

**Check-only mode** (`--check-only`):

1. Open the rosbag as usual
2. Read all messages — for each matching topic:
   - Collect nanosecond timestamp (`stamp_ns`) into a list
   - **Skip** JPEG decode, `cv2.imwrite`, IMU file write
   - Still print per-message status line (count + timestamp) for progress feedback
3. After the loop, compute and print the report (see below)

### Report format

```
=== Intra-Topic Interval Consistency ===

Image stream (/image/compressed):
  Count:              1585
  Mean interval:      33.33 ms  (expected 30 Hz)
  Interval stats:
    min/mean/max:     17.2 / 33.3 / 53.2 ms
    median/std:       33.2 / 2.1 ms
  Within ±1%  of mean:  94.2%  (1492 / 1585)
  Within ±5%  of mean:  98.7%  (1565 / 1585)
  Within ±10% of mean:  99.4%  (1576 / 1585)

IMU stream (/imu/data):
  Count:              10547
  Mean interval:      5.00 ms  (expected 200 Hz)
  Interval stats:
    min/mean/max:     0.1 / 5.0 / 10.9 ms
    median/std:       5.0 / 0.5 ms
  Within ±1%  of mean:  88.3%  (9312 / 10547)
  Within ±5%  of mean:  97.1%  (10242 / 10547)
  Within ±10% of mean:  98.9%  (10432 / 10547)

=== Cross-Topic Synchronization ===

Image-to-nearest-IMU offset:
  Count:              1585
  min/mean/max:       0.0 / 1.2 / 2.8 ms
  median/std:         1.1 / 0.6 ms
```

### Computation details

**Intra-topic intervals:**
- `intervals_ms = np.diff(timestamps_ns) / 1e6`
- Expected rate computed as `1 / mean_interval_s` and printed in Hz for context

**Interval distribution (1%, 5%, 10%):**
- For each interval `d` and mean `m`: check `abs(d - m) / m <= threshold`
- Print count and percentage for each threshold

**Cross-topic offset:**
- For each image timestamp `t_img`, find the nearest IMU timestamp using `np.searchsorted` on the sorted IMU array
- Compute absolute offset: `abs(t_img - nearest_imu) / 1e6` (in ms)
- Aggregate: min/max/mean/median/std

### Control flow in main()

```
if args.check_only:
    # Read loop: collect timestamps only
    # After loop: compute and print report
    return 0
else:
    # Existing decode-and-save path (unchanged)
```

The two modes share the bag-opening and topic-type-map code. The read loop is the main branch point — in check-only mode it's a tight loop collecting timestamps; in normal mode it does full decode and file I/O.

### Files

| File | Change |
|------|--------|
| `scripts/decode_rosbag.py` | Add `--check-only` arg, check-only read loop, report functions |

### Testing

- Run `--check-only` against `recordings/rosbag2_2026_07_12-04_25_53/` → verify report prints, no files created
- Verify image count (1585) and IMU count (10547) in report match `ros2 bag info`
- Cross-check: mean image interval ~33.3 ms, mean IMU interval ~5.0 ms
- Regression: normal mode (without `--check-only`) still produces JPEGs + `imu_data.txt`
