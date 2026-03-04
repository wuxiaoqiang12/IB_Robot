"""Leader arm publisher launch file for SO-101 robot arm."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for leader arm publisher."""
    
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for motor communication'
    )
    
    calib_file_arg = DeclareLaunchArgument(
        'calib_file',
        default_value='',
        description='Path to calibration file'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Joint state publishing rate (Hz)'
    )

    # Leader arm publisher node
    leader_pub_node = Node(
        package='so101_hardware',
        executable='leader_arm_pub',
        name='so101_leader_publisher',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'calib_file': LaunchConfiguration('calib_file'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }]
    )

    return LaunchDescription([
        port_arg,
        calib_file_arg,
        publish_rate_arg,
        leader_pub_node,
    ])