#!/usr/bin/env python3
"""
Decode a ROS2 rosbag (rosbag2 sqlite3 .db3): extract stereo images as JPEG
and IMU data as text.

Reads the .db3 file(s) directly (no metadata.yaml required), so it works on
bags that only carry the sqlite3 storage file.

Usage:
    python3 decode_rosbag.py --bag-dir /path/to/rosbag2_2026_09_20-07_55_08 \
        --output-dir ./recordings \
        --left-topic /image/left --right-topic /image/right \
        --imu-topic /imu/data
"""

import argparse
import glob
import os
import sqlite3
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, Imu


def _print_stream_stats(timestamps, count, label):
    """Print statistics for a single stream (same format as validate_topics.py)."""
    if count == 0:
        print(f'No {label} data received.')
        return

    total_time = timestamps[-1] - timestamps[0]
    actual_rate = (count - 1) / total_time if total_time > 0 else 0.0

    if count > 1:
        intervals = np.diff(timestamps) * 1000.0  # convert to ms
        print(f'Total frames:      {count}')
        print(f'Actual rate:       {actual_rate:.1f} Hz')
        print(f'Interval min/mean/max: '
              f'{intervals.min():.1f} / {intervals.mean():.1f} / '
              f'{intervals.max():.1f} ms')
    else:
        print(f'Total frames:      {count}')
        print(f'Actual rate:       N/A (only 1 sample)')
        print(f'Interval min/mean/max: N/A (only 1 sample)')
    print(f'First stamp:       {timestamps[0]:.3f}')
    print(f'Last stamp:        {timestamps[-1]:.3f}')
    print(f'Total duration:    {total_time:.1f} s')


def _print_summary(camera_streams, imu_timestamps, imu_count,
                   imu_file, output_dir):
    """Print statistics and close IMU file."""
    # Close IMU text file if open
    if imu_file is not None:
        imu_file.close()
        imu_path = os.path.join(output_dir, 'imu_data.txt')
        print(f'\nIMU data saved to: {imu_path}')

    print()
    print('=' * 60)
    for stream in camera_streams:
        print(f'=== {stream["name"].capitalize()} Image Statistics ===')
        _print_stream_stats(stream['timestamps'], stream['count'],
                            f'{stream["name"]} image')
        print()

    print('=== IMU Statistics ===')
    _print_stream_stats(imu_timestamps, imu_count, 'IMU')

    print(f'\nSaved images to: {output_dir}')
    print('=' * 60)


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


def _print_sync_report(image_timestamps_ns, imu_timestamps_ns, name=''):
    """Print cross-topic image-to-nearest-IMU offset statistics."""
    if len(image_timestamps_ns) == 0:
        print(f'\nNo {name + " " if name else ""}image data '
              f'for synchronization analysis.')
        return
    if len(imu_timestamps_ns) == 0:
        print('\nNo IMU data for synchronization analysis.')
        return

    imu_array = np.array(imu_timestamps_ns)
    offsets_ms = []       # absolute offset
    signed_offsets_ms = []  # signed offset: positive = IMU after image
    for t_img in image_timestamps_ns:
        idx = np.searchsorted(imu_array, t_img)
        # Check nearest neighbor (idx-1 and idx, clamped to bounds)
        candidate_idxs = []
        if idx > 0:
            candidate_idxs.append(idx - 1)
        if idx < len(imu_array):
            candidate_idxs.append(idx)
        best_idx = min(candidate_idxs,
                       key=lambda i: abs(t_img - imu_array[i]))
        offset_ns = imu_array[best_idx] - t_img  # signed: positive = IMU after image
        signed_offsets_ms.append(offset_ns / 1e6)
        offsets_ms.append(abs(offset_ns) / 1e6)

    offsets = np.array(offsets_ms)
    signed = np.array(signed_offsets_ms)

    header = (f'\n=== Cross-Topic Synchronization ({name}) ==='
              if name else f'\n=== Cross-Topic Synchronization ===')
    print(header)

    # --- Absolute offset summary ---
    print(f'\nAbsolute Image-to-nearest-IMU offset:')
    print(f'  Count:              {len(offsets)}')
    print(f'  min / mean / max:   {offsets.min():.2f} / '
          f'{offsets.mean():.2f} / {offsets.max():.2f} ms')
    print(f'  median / std:       {np.median(offsets):.2f} / '
          f'{offsets.std():.2f} ms')

    # --- Signed offset summary (directional bias) ---
    imu_after = np.sum(signed > 0)
    imu_before = np.sum(signed < 0)
    exact = np.sum(signed == 0)
    print(f'\nDirectional bias (signed offset, + = IMU after image):')
    print(f'  IMU after image:    {imu_after} ({imu_after / len(signed) * 100:.1f}%)')
    print(f'  IMU before image:   {imu_before} ({imu_before / len(signed) * 100:.1f}%)')
    print(f'  Exact match:        {exact} ({exact / len(signed) * 100:.1f}%)')
    print(f'  Signed mean:        {signed.mean():+.2f} ms')
    print(f'  Signed median:      {np.median(signed):+.2f} ms')
    print(f'  Signed range:       [{signed.min():+.2f}, {signed.max():+.2f}] ms')

    # --- Percentile distribution ---
    percentiles = [50, 75, 90, 95, 99, 99.9]
    pct_values = np.percentile(offsets, percentiles)
    print(f'\nAbsolute offset percentiles:')
    for p, v in zip(percentiles, pct_values):
        print(f'  P{p:5.1f}:            {v:.2f} ms')

    # --- Coverage summary: cumulative fraction within key thresholds ---
    thresholds_ms = [0.1, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0]
    print(f'\nCumulative coverage (absolute offset):')
    for t in thresholds_ms:
        if t > offsets.max():
            break
        pct = np.sum(offsets <= t) / len(offsets) * 100.0
        print(f'  ≤ {t:5.1f} ms:         {pct:5.1f}%')

    # --- Signed offset ASCII histogram ---
    signed_min = signed.min()
    signed_max = signed.max()
    # Use evenly-spaced bins across the signed range
    n_bins = 20
    counts, bin_edges = np.histogram(signed, bins=n_bins)
    max_count = counts.max()
    bar_width = 60 if max_count > 0 else 1

    print(f'\nSigned offset histogram (ms) [+ = IMU after image]:')
    for i in range(len(counts)):
        if counts[i] == 0:
            continue
        bar_len = int(counts[i] / max_count * bar_width)
        pct = counts[i] / len(signed) * 100.0
        bar = '#' * bar_len
        print(f'  {bin_edges[i]:+7.2f} - '
              f'{bin_edges[i+1]:+7.2f} | {bar:<{bar_width}} '
              f'{counts[i]:>5} ({pct:5.1f}%)')


def _write_timestamp_files(camera_streams, imu_timestamps_ns, output_dir):
    """Save raw timestamps to text files (one ns integer per line, no header)."""
    for stream in camera_streams:
        filepath = os.path.join(output_dir,
                                f'{stream["name"]}_timestamps.txt')
        with open(filepath, 'w') as f:
            for ts in stream['timestamps_ns']:
                f.write(f'{ts}\n')
        print(f'{stream["name"].capitalize()} timestamps saved to: {filepath}')

    filepath = os.path.join(output_dir, 'imu_timestamps.txt')
    with open(filepath, 'w') as f:
        for ts in imu_timestamps_ns:
            f.write(f'{ts}\n')
    print(f'IMU timestamps saved to: {filepath}')


def _print_check_report(camera_streams, imu_timestamps_ns,
                        imu_topic, output_dir):
    """Print the full timestamp consistency report."""
    print()
    print('=' * 60)
    print('=== Timestamp Consistency Report ===')

    print('\n=== Intra-Topic Interval Consistency ===')
    for stream in camera_streams:
        _print_interval_report(f'{stream["name"].capitalize()} image stream',
                               stream['topic'], stream['timestamps_ns'])
    _print_interval_report('IMU stream', imu_topic, imu_timestamps_ns)

    for stream in camera_streams:
        _print_sync_report(stream['timestamps_ns'], imu_timestamps_ns,
                           name=stream['name'])

    if output_dir is not None:
        print()
        _write_timestamp_files(camera_streams, imu_timestamps_ns, output_dir)

    print()
    print('=' * 60)


def _decode_image(msg):
    """Convert a sensor_msgs/Image into a BGR numpy array for cv2.imwrite.

    Returns None if the encoding is unsupported or the data is malformed.
    """
    if msg.is_bigendian:
        print(f'[IMG] Warning: big-endian image not supported '
              f'(encoding {msg.encoding}), skipping', file=sys.stderr)
        return None

    if msg.encoding in ('bgr8', '8UC3'):
        channels = 3
        convert = None
    elif msg.encoding == 'rgb8':
        channels = 3
        convert = cv2.COLOR_RGB2BGR
    elif msg.encoding in ('mono8', '8UC1'):
        channels = 1
        convert = None
    else:
        print(f'[IMG] Warning: unsupported encoding {msg.encoding}, '
              f'skipping', file=sys.stderr)
        return None

    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    row_bytes = msg.width * channels
    expected = row_bytes * msg.height
    if data.size < expected:
        print(f'[IMG] Warning: data too short for {msg.width}x{msg.height} '
              f'({msg.encoding}): got {data.size} bytes, expected {expected}, '
              f'skipping', file=sys.stderr)
        return None

    if msg.step == row_bytes:
        arr = data[:expected].reshape(msg.height, msg.width, channels)
    else:
        # Row stride includes padding: copy each row's payload out.
        arr = np.zeros((msg.height, msg.width, channels), dtype=np.uint8)
        for y in range(msg.height):
            row = data[y * msg.step:y * msg.step + row_bytes]
            arr[y] = row.reshape(msg.width, channels)

    if channels == 1:
        arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    elif convert is not None:
        arr = cv2.cvtColor(arr, convert)
    return arr


def _read_topic_type_map(bag_dir):
    """Read topic name -> type across all .db3 files in the bag directory."""
    topic_type_map = {}
    for db3_path in sorted(glob.glob(os.path.join(bag_dir, '*.db3'))):
        con = sqlite3.connect(f'{Path(db3_path).as_uri()}?mode=ro', uri=True)
        try:
            for name, typ in con.execute('SELECT name, type FROM topics'):
                topic_type_map.setdefault(name, typ)
        finally:
            con.close()
    return topic_type_map


def _iter_messages(bag_dir):
    """Yield (topic_name, topic_type, raw_data) across all .db3 files in order."""
    for db3_path in sorted(glob.glob(os.path.join(bag_dir, '*.db3'))):
        con = sqlite3.connect(f'{Path(db3_path).as_uri()}?mode=ro', uri=True)
        try:
            topic_map = {}
            for tid, name, typ in con.execute(
                    'SELECT id, name, type FROM topics'):
                topic_map[tid] = (name, typ)

            for tid, raw_data in con.execute(
                    'SELECT topic_id, data FROM messages ORDER BY id'):
                name, typ = topic_map.get(tid, ('', ''))
                yield name, typ, raw_data
        finally:
            con.close()


def main():
    parser = argparse.ArgumentParser(
        description='Decode a ROS2 rosbag: extract stereo images and IMU data')
    parser.add_argument(
        '--bag-dir', required=True,
        help='Path to rosbag directory (contains one or more .db3 files)')
    parser.add_argument(
        '--output-dir', default='./recordings',
        help='Parent output directory — timestamped subfolder created inside '
             '(default: ./recordings)')
    parser.add_argument(
        '--left-topic', default='/image/left',
        help='Left image topic name in the bag (default: /image/left)')
    parser.add_argument(
        '--right-topic', default='/image/right',
        help='Right image topic name in the bag (default: /image/right)')
    parser.add_argument(
        '--imu-topic', default='/imu/data',
        help='IMU topic name in the bag (default: /imu/data)')
    parser.add_argument(
        '--check-only', action='store_true', default=False,
        help='Validate timestamps only; do not save data')
    args = parser.parse_args()

    # Validate bag directory
    if not os.path.isdir(args.bag_dir):
        print(f'Error: bag directory not found: {args.bag_dir}', file=sys.stderr)
        return 1
    if not glob.glob(os.path.join(args.bag_dir, '*.db3')):
        print(f'Error: no .db3 file found in: {args.bag_dir}', file=sys.stderr)
        return 1

    # Topic name -> type map read directly from the sqlite3 storage
    topic_type_map = _read_topic_type_map(args.bag_dir)
    print(f'Bag contains topics: {topic_type_map}')

    # Camera streams (left + right), keyed by topic for dispatch
    camera_streams = []
    topic_to_stream = {}
    for name, topic in (('left', args.left_topic), ('right', args.right_topic)):
        stream = {'name': name, 'topic': topic,
                  'timestamps': [], 'timestamps_ns': [], 'count': 0}
        camera_streams.append(stream)
        topic_to_stream[topic] = stream

    imu_count = 0
    imu_timestamps = []
    imu_timestamps_ns = []
    warned_topics = set()

    def _handle_image(stream, raw_msg):
        stream['count'] += 1
        msg = deserialize_message(raw_msg, Image)
        # Use the sensor timestamp from the message header, not the bag-record time
        header_stamp_ns = (msg.header.stamp.sec * 1_000_000_000
                           + msg.header.stamp.nanosec)
        stream['timestamps'].append(header_stamp_ns / 1e9)
        stream['timestamps_ns'].append(header_stamp_ns)

        arr = _decode_image(msg)
        if arr is None:
            return

        filename = (f'{stream["name"]}_{stream["count"]:04d}_'
                    f'{header_stamp_ns}.jpg')
        cv2.imwrite(os.path.join(output_dir, filename), arr)
        print(f'[{stream["name"].upper()} #{stream["count"]:04d}] '
              f'stamp={header_stamp_ns / 1e9:.3f} '
              f'size={arr.shape[1]}x{arr.shape[0]}')

    def _handle_imu(raw_msg):
        nonlocal imu_count
        imu_count += 1
        msg = deserialize_message(raw_msg, Imu)
        # Use the sensor timestamp from the message header, not the bag-record time
        header_stamp_ns = (msg.header.stamp.sec * 1_000_000_000
                           + msg.header.stamp.nanosec)
        imu_timestamps.append(header_stamp_ns / 1e9)
        imu_timestamps_ns.append(header_stamp_ns)

        if args.check_only:
            return

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # Open IMU text file on first message
        if imu_file[0] is None:
            filepath = os.path.join(output_dir, 'imu_data.txt')
            imu_file[0] = open(filepath, 'w')
            imu_file[0].write('# timestamp_ns gx gy gz ax ay az\n')

        imu_file[0].write(
            f'{header_stamp_ns} {gx:.4f} {gy:.4f} {gz:.4f} '
            f'{ax:.4f} {ay:.4f} {az:.4f}\n')
        imu_file[0].flush()

        print(f'[IMU #{imu_count:04d}] stamp={header_stamp_ns / 1e9:.3f} '
              f'accel=({ax:.2f},{ay:.2f},{az:.2f}) '
              f'gyro=({gx:.2f},{gy:.2f},{gz:.2f})')

    # Create timestamped output directory (used by both modes)
    timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = os.path.join(args.output_dir, timestamp_str)
    os.makedirs(output_dir, exist_ok=True)

    if args.check_only:
        print('Reading messages (check-only mode)...')
    else:
        print(f'Decoding to: {output_dir}')

    imu_file = [None]  # mutable holder so the nested handler can set it

    for topic_name, msg_type, raw_msg in _iter_messages(args.bag_dir):
        if topic_name in topic_to_stream:
            if 'Image' not in msg_type:
                print(f'Warning: unexpected type {msg_type} '
                      f'on topic {topic_name}, skipping', file=sys.stderr)
                continue
            _handle_image(topic_to_stream[topic_name], raw_msg)
        elif topic_name == args.imu_topic:
            if 'Imu' not in msg_type:
                print(f'Warning: unexpected type {msg_type} '
                      f'on topic {topic_name}, skipping', file=sys.stderr)
                continue
            _handle_imu(raw_msg)
        else:
            if topic_name not in warned_topics:
                print(f'Warning: skipping unknown topic "{topic_name}"',
                      file=sys.stderr)
                warned_topics.add(topic_name)

    if args.check_only:
        per_camera = '; '.join(f'{s["name"]}: {s["count"]} images'
                               for s in camera_streams)
        print(f'Done. {per_camera}, {imu_count} IMU samples.')
        _print_check_report(camera_streams, imu_timestamps_ns,
                            args.imu_topic, output_dir)
        return 0

    _print_summary(camera_streams, imu_timestamps, imu_count,
                   imu_file[0], output_dir)
    return 0


if __name__ == '__main__':
    sys.exit(main())
