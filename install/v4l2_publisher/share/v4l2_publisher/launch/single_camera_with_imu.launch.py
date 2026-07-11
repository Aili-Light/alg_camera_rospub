from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # --- Camera parameters ---
        DeclareLaunchArgument('device_path', default_value='/dev/video0'),
        DeclareLaunchArgument('requested_width', default_value='1280'),
        DeclareLaunchArgument('requested_height', default_value='800'),
        DeclareLaunchArgument('target_width', default_value='640'),
        DeclareLaunchArgument('target_height', default_value='400'),
        DeclareLaunchArgument('pixel_format', default_value='YUYV'),
        DeclareLaunchArgument('fps', default_value='60.0'),
        DeclareLaunchArgument('topic_name', default_value='/image'),
        DeclareLaunchArgument('frame_id', default_value='camera_link'),
        DeclareLaunchArgument('publish_rectified', default_value='false'),
        DeclareLaunchArgument('calib_path', default_value='./camera_calib.yaml'),

        # --- IMU parameters ---
        DeclareLaunchArgument('i2c_bus', default_value='/dev/i2c-3'),
        DeclareLaunchArgument('imu_address', default_value='105'),
        DeclareLaunchArgument('imu_frequency', default_value='100.0'),
        DeclareLaunchArgument('imu_topic_name', default_value='imu/data'),
        DeclareLaunchArgument('imu_frame_id', default_value='imu_link'),

        # --- Single camera node ---
        Node(
            package='v4l2_publisher',
            executable='single_camera_publisher',
            name='single_camera_publisher',
            output='screen',
            parameters=[{
                'device_path': LaunchConfiguration('device_path'),
                'requested_width': LaunchConfiguration('requested_width'),
                'requested_height': LaunchConfiguration('requested_height'),
                'target_width': LaunchConfiguration('target_width'),
                'target_height': LaunchConfiguration('target_height'),
                'pixel_format': LaunchConfiguration('pixel_format'),
                'fps': LaunchConfiguration('fps'),
                'topic_name': LaunchConfiguration('topic_name'),
                'frame_id': LaunchConfiguration('frame_id'),
                'publish_rectified': LaunchConfiguration('publish_rectified'),
                'calib_path': LaunchConfiguration('calib_path'),
            }],
        ),

        # --- IMU node ---
        Node(
            package='v4l2_publisher',
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
    ])
