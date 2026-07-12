# Design: Offset Histogram + Timestamp Dump for --check-only Mode

**Date:** 2026-07-12
**Status:** Draft
**Package:** `alg_camera_rospub`

## Motivation

The `--check-only` mode currently prints aggregate offset stats but doesn't show the distribution shape. A histogram would reveal bimodality, long tails, or clustering patterns. Additionally, raw timestamp dumps enable external analysis tools to compute custom metrics.

## Goals

- Print an ASCII histogram of image-to-nearest-IMU offset distribution
- Save image timestamps to `{output_dir}/YYYYMMDD_HHMMSS/image_timestamps.txt`
- Save IMU timestamps to `{output_dir}/YYYYMMDD_HHMMSS/imu_timestamps.txt`
- Timestamp files: one nanosecond integer per line, no header
- All outputs only in `--check-only` mode

## Non-Goals

- Configurable bin count (fixed at 20)
- Other output formats (CSV, binary)

## Design

All changes in `scripts/decode_rosbag.py`.

### 1. ASCII histogram

Added at the end of `_print_sync_report()`. Uses `np.histogram` with 20 bins.

```
Offset histogram (ms):
  0.0 -  4.1 | ########################################        1234 (77.9%)
  4.1 -  8.2 | #####                                             89 ( 5.6%)
  8.2 - 12.3 | ###                                               56 ( 3.5%)
  ...
```

- Each `#` represents one count
- Bar width capped at 60 characters; count label and percentage right-aligned
- Bin edges printed with 1 decimal place

### 2. Timestamp text files

In `--check-only` mode, after collecting timestamps, write two text files to a timestamped subfolder:

```
{output_dir}/YYYYMMDD_HHMMSS/image_timestamps.txt
{output_dir}/YYYYMMDD_HHMMSS/imu_timestamps.txt
```

Content: one nanosecond integer per line, no header.

### Control flow

```
if args.check_only:
    # Read loop: collect timestamps (unchanged)
    # Create output_dir only when timestamps need saving
    # Write image_timestamps.txt, imu_timestamps.txt
    # Print report (including new histogram)
    return 0
```

### API changes

| Function | Change |
|----------|--------|
| `_print_sync_report()` | Add histogram section at end |
| `_print_check_report()` | Accept new `output_dir` param, call `_write_timestamp_files()` |
| `_write_timestamp_files()` | New: write two text files |

### Files

| File | Change |
|------|--------|
| `scripts/decode_rosbag.py` | ~40 lines added |

### Testing

- Run `--check-only` against `recordings/rosbag2_2026_07_12-04_25_53/`
- Verify histogram prints with 20 bins
- Verify `image_timestamps.txt` has 1585 lines (one per image)
- Verify `imu_timestamps.txt` has 10547 lines (one per IMU sample)
- Verify regression: normal mode unchanged
