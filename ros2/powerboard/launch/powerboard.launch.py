from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='port',
            # by-id rather than /dev/ttyACM0: ACM numbering shifts when another USB
            # serial device enumerates, and this NUC already has an Xsens IMU.
            default_value='/dev/serial/by-id/usb-Raspberry_Pi_Pico_2_*-if00',
            description='Serial port of the powerboard; glob patterns are allowed.'),
        DeclareLaunchArgument(
            name='namespace', default_value='freya_1',
            description='Namespace for the node and its topics.'),
        DeclareLaunchArgument(
            name='warn_fraction', default_value='0.9',
            description='Fraction of a branch limit at which to warn (0-1).'),

        Node(
            package='powerboard',
            executable='powerboard_node',
            name='powerboard',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            emulate_tty=True,
            respawn=True,
            respawn_delay=5.0,
            parameters=[{
                'port': LaunchConfiguration('port'),
                'warn_fraction': LaunchConfiguration('warn_fraction'),
            }],
        ),
    ])
