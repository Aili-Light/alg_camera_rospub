from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    imu_executable = PathJoinSubstitution([
        FindPackageShare('v4l2_publisher'), '..', '..', 'lib', 'v4l2_publisher', 'imu_ros2_pub_test'
    ])
    imu_library_path = PathJoinSubstitution([
        FindPackageShare('v4l2_publisher'), '..', '..', 'lib', 'v4l2_publisher'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('left_topic', default_value='/image/left'),
        DeclareLaunchArgument('right_topic', default_value='/image/right'),
        DeclareLaunchArgument('publish_rectified_images', default_value='false'),
        DeclareLaunchArgument('camera_ready_timeout_sec', default_value='30.0'),
        DeclareLaunchArgument('i2c_bus', default_value='/dev/i2c-3'),
        DeclareLaunchArgument('imu_address', default_value='105'),
        DeclareLaunchArgument('raw_imu_topic', default_value='_imu/data_raw'),
        DeclareLaunchArgument('topic_name', default_value='imu/data'),
        DeclareLaunchArgument('frequency', default_value='200.0'),
        Node(
            package='v4l2_publisher',
            executable='stereo_image_publisher',
            name='stereo_image_publisher',
            output='screen',
            parameters=[{
                'publish_rectified_images': LaunchConfiguration('publish_rectified_images'),
            }],
        ),
        Node(
            package='v4l2_publisher',
            executable='start_imu_after_camera_ready.py',
            name='camera_ready_gate',
            output='screen',
            parameters=[{
                'left_topic': LaunchConfiguration('left_topic'),
                'right_topic': LaunchConfiguration('right_topic'),
                'timeout_sec': LaunchConfiguration('camera_ready_timeout_sec'),
                'imu_executable': imu_executable,
                'imu_ld_library_path': imu_library_path,
                'i2c_bus': LaunchConfiguration('i2c_bus'),
                'imu_address': LaunchConfiguration('imu_address'),
                'topic_name': LaunchConfiguration('raw_imu_topic'),
                'frequency': LaunchConfiguration('frequency'),
                'output_imu_topic': LaunchConfiguration('topic_name'),
            }],
        ),
    ])
