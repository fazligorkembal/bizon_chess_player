"""Live view of the bench IMU: serial stream, orientation filter, RViz."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('bizon_imu'))
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        # Respawn because reflashing the board or a loose USB cable ends the node, and the
        # view should come back on its own when the port does.
        Node(
            package='bizon_imu',
            executable='serial_imu_node',
            parameters=[{'port': LaunchConfiguration('port')}],
            output='screen',
            respawn=True,
            respawn_delay=2.0,
        ),
        # The GY-521 has no magnetometer, so yaw comes from the gyro alone and has no absolute
        # reference. constant_dt is the firmware's sample period: message stamps are taken on
        # arrival, and USB delivers samples in bursts.
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            parameters=[{
                'use_mag': False,
                'world_frame': 'enu',
                'fixed_frame': 'imu_world',
                'publish_tf': True,
                'constant_dt': 0.001,
            }],
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', str(share / 'rviz' / 'imu.rviz')],
            output='screen',
        ),
    ])
