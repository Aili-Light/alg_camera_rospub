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


def _print_summary(image_timestamps, img_count, imu_timestamps, imu_count,
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
    parser.add_argument(
        '--check-only', action='store_true', default=False,
        help='Validate timestamps only; do not save data')
    args = parser.parse_args()

    # Validate bag directory
    if not os.path.isdir(args.bag_dir):
        print(f'Error: bag directory not found: {args.bag_dir}', file=sys.stderr)
        return 1

    # Open the rosbag
    storage_options = StorageOptions(uri=args.bag_dir, storage_id='sqlite3')
    converter_options = ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Topic name → type map from bag metadata
    topic_type_map = {}
    for topic_meta in reader.get_all_topics_and_types():
        topic_type_map[topic_meta.name] = topic_meta.type

    print(f'Bag contains topics: {topic_type_map}')

    # State (used by both modes)
    img_count = 0
    imu_count = 0

    if args.check_only:
        # --- Check-only mode: collect timestamps, print report ---
        image_timestamps_ns = []
        imu_timestamps_ns = []

        print('Reading messages (check-only mode)...')
        while reader.has_next():
            (topic_name, raw_msg, _stamp_ns) = reader.read_next()

            if topic_name == args.camera_topic:
                msg = deserialize_message(raw_msg, CompressedImage)
                header_stamp_ns = (msg.header.stamp.sec * 1_000_000_000
                                   + msg.header.stamp.nanosec)
                image_timestamps_ns.append(header_stamp_ns)
                img_count += 1
                if img_count % 100 == 0:
                    print(f'  ... {img_count} images, '
                          f'{imu_count} IMU samples')

            elif topic_name == args.imu_topic:
                msg = deserialize_message(raw_msg, Imu)
                header_stamp_ns = (msg.header.stamp.sec * 1_000_000_000
                                   + msg.header.stamp.nanosec)
                imu_timestamps_ns.append(header_stamp_ns)
                imu_count += 1
                if imu_count % 2000 == 0:
                    print(f'  ... {img_count} images, '
                          f'{imu_count} IMU samples')

        print(f'Done. {img_count} images, {imu_count} IMU samples.')

        # Create timestamped output directory for timestamp files
        timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
        check_output_dir = os.path.join(args.output_dir, timestamp_str)
        os.makedirs(check_output_dir, exist_ok=True)

        _print_check_report(image_timestamps_ns, imu_timestamps_ns,
                           args.camera_topic, args.imu_topic,
                           check_output_dir)
        return 0

    # --- Normal mode: decode and save (existing code) ---
    # Create timestamped output directory
    timestamp_str = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_dir = os.path.join(args.output_dir, timestamp_str)
    os.makedirs(output_dir, exist_ok=True)
    print(f'Decoding to: {output_dir}')

    # State
    image_timestamps = []
    imu_timestamps = []
    img_count = 0
    imu_count = 0
    imu_file = None

    # Read loop
    while reader.has_next():
        (topic_name, raw_msg, _stamp_ns) = reader.read_next()

        if topic_name == args.camera_topic:
            msg_type = topic_type_map.get(topic_name, '')
            if 'CompressedImage' not in msg_type:
                print(f'Warning: unexpected type {msg_type} '
                      f'on topic {topic_name}, skipping', file=sys.stderr)
                continue

            msg = deserialize_message(raw_msg, CompressedImage)
            img_count += 1
            # Use the sensor timestamp from the message header, not the bag-record time
            header_stamp_ns = (msg.header.stamp.sec * 1_000_000_000
                               + msg.header.stamp.nanosec)
            stamp_sec = header_stamp_ns / 1e9
            image_timestamps.append(stamp_sec)

            # Decode JPEG bytes
            jpeg_bytes = np.frombuffer(msg.data, dtype=np.uint8)
            arr = cv2.imdecode(jpeg_bytes, cv2.IMREAD_COLOR)

            if arr is None:
                print(f'[IMG #{img_count:04d}] ERROR: failed to decode '
                      f'compressed image', file=sys.stderr)
                continue

            # Save as JPEG
            filename = f'image_{img_count:04d}_{header_stamp_ns}.jpg'
            filepath = os.path.join(output_dir, filename)
            cv2.imwrite(filepath, arr)

            print(f'[IMG #{img_count:04d}] stamp={stamp_sec:.3f} '
                  f'size={arr.shape[1]}x{arr.shape[0]} compressed')

        elif topic_name == args.imu_topic:
            msg_type = topic_type_map.get(topic_name, '')
            if 'Imu' not in msg_type:
                print(f'Warning: unexpected type {msg_type} '
                      f'on topic {topic_name}, skipping', file=sys.stderr)
                continue

            msg = deserialize_message(raw_msg, Imu)
            imu_count += 1
            # Use the sensor timestamp from the message header, not the bag-record time
            header_stamp_ns = (msg.header.stamp.sec * 1_000_000_000
                               + msg.header.stamp.nanosec)
            stamp_sec = header_stamp_ns / 1e9
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
                f'{header_stamp_ns} {gx:.4f} {gy:.4f} {gz:.4f} '
                f'{ax:.4f} {ay:.4f} {az:.4f}\n')
            imu_file.flush()

            print(f'[IMU #{imu_count:04d}] stamp={stamp_sec:.3f} '
                  f'accel=({ax:.2f},{ay:.2f},{az:.2f}) '
                  f'gyro=({gx:.2f},{gy:.2f},{gz:.2f})')

        else:
            print(f'Warning: skipping unknown topic "{topic_name}"', file=sys.stderr)

    # Print summary
    _print_summary(image_timestamps, img_count, imu_timestamps, imu_count,
                  imu_file, output_dir)
    return 0


if __name__ == '__main__':
    sys.exit(main())
