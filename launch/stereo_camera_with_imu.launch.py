from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # --- Left camera parameters ---
        DeclareLaunchArgument('left_device_path', default_value='/dev/video0'),
        DeclareLaunchArgument('left_requested_width', default_value='1280'),
        DeclareLaunchArgument('left_requested_height', default_value='800'),
        DeclareLaunchArgument('left_target_width', default_value='1280'),
        DeclareLaunchArgument('left_target_height', default_value='800'),
        DeclareLaunchArgument('left_pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('left_fps', default_value='60.0'),
        DeclareLaunchArgument('left_topic_name', default_value='/image/left'),
        DeclareLaunchArgument('left_frame_id', default_value='left_camera_link'),

        # --- Right camera parameters ---
        DeclareLaunchArgument('right_device_path', default_value='/dev/video1'),
        DeclareLaunchArgument('right_requested_width', default_value='1280'),
        DeclareLaunchArgument('right_requested_height', default_value='800'),
        DeclareLaunchArgument('right_target_width', default_value='1280'),
        DeclareLaunchArgument('right_target_height', default_value='800'),
        DeclareLaunchArgument('right_pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('right_fps', default_value='60.0'),
        DeclareLaunchArgument('right_topic_name', default_value='/image/right'),
        DeclareLaunchArgument('right_frame_id', default_value='right_camera_link'),

        # --- Color and compression ---
        DeclareLaunchArgument('left_output_color', default_value='true'),
        DeclareLaunchArgument('right_output_color', default_value='true'),
        DeclareLaunchArgument('publish_compressed', default_value='false'),
        DeclareLaunchArgument('compressed_quality', default_value='80'),

        # --- IMU parameters ---
        DeclareLaunchArgument('i2c_bus', default_value='/dev/i2c-3'),
        DeclareLaunchArgument('imu_address', default_value='105'),
        DeclareLaunchArgument('imu_frequency', default_value='100.0'),
        DeclareLaunchArgument('imu_topic_name', default_value='imu/data'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link'),

        # --- IMU start delay ---
        DeclareLaunchArgument('imu_start_delay', default_value='2.0'),

        # --- Stereo camera node (starts immediately) ---
        Node(
            package='alg_camera_rospub',
            executable='stereo_camera_publisher',
            name='stereo_camera_publisher',
            output='screen',
            parameters=[{
                'left_device_path': LaunchConfiguration('left_device_path'),
                'left_requested_width': LaunchConfiguration('left_requested_width'),
                'left_requested_height': LaunchConfiguration('left_requested_height'),
                'left_target_width': LaunchConfiguration('left_target_width'),
                'left_target_height': LaunchConfiguration('left_target_height'),
                'left_pixel_format': LaunchConfiguration('left_pixel_format'),
                'left_fps': LaunchConfiguration('left_fps'),
                'left_topic_name': LaunchConfiguration('left_topic_name'),
                'left_frame_id': LaunchConfiguration('left_frame_id'),
                'left_output_color': LaunchConfiguration('left_output_color'),
                'right_device_path': LaunchConfiguration('right_device_path'),
                'right_requested_width': LaunchConfiguration('right_requested_width'),
                'right_requested_height': LaunchConfiguration('right_requested_height'),
                'right_target_width': LaunchConfiguration('right_target_width'),
                'right_target_height': LaunchConfiguration('right_target_height'),
                'right_pixel_format': LaunchConfiguration('right_pixel_format'),
                'right_fps': LaunchConfiguration('right_fps'),
                'right_topic_name': LaunchConfiguration('right_topic_name'),
                'right_frame_id': LaunchConfiguration('right_frame_id'),
                'right_output_color': LaunchConfiguration('right_output_color'),
                'publish_compressed': LaunchConfiguration('publish_compressed'),
                'compressed_quality': LaunchConfiguration('compressed_quality'),
            }],
        ),

        # --- IMU node (delayed until camera streaming enables I2C bus) ---
        TimerAction(
            period=LaunchConfiguration('imu_start_delay'),
            actions=[
                Node(
                    package='alg_camera_rospub',
                    executable='imu_publisher',
                    name='imu_publisher',
                    output='screen',
                    parameters=[{
                        'i2c_bus': LaunchConfiguration('i2c_bus'),
                        'imu_address': LaunchConfiguration('imu_address'),
                        'frequency': LaunchConfiguration('imu_frequency'),
                        'topic_name': LaunchConfiguration('imu_topic_name'),
                        'frame_id': LaunchConfiguration('imu_frame_id'),
                    }],
                ),
            ],
        ),
    ])
