"""Single entry point for one robot.

Replaces the three terminals per robot documented in the README
(bizon_player_bringup, bizon_lifecycle_dev, and the behavior tree client)
with one launch file. The three underlying launch files are untouched and can
still be run individually when debugging one layer in isolation.

    ros2 launch bizon_player_bringup bizon_player.launch.py prefix:=bizon2
    ros2 launch bizon_player_bringup bizon_player.launch.py prefix:=bizon3

Startup order is enforced with timers rather than event handlers on purpose.
An OnProcessStart handler fires when a process is spawned, not when it is ready
to serve: ArmPlugin builds a MoveGroupInterface during configure, which needs
move_group already answering. Until a wait-for-service helper exists, the delays
below are the honest mechanism, and both are launch arguments so they can be
raised on a slower machine.
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Behavior parameters differ per robot. Picking the file from the prefix keeps
# the caller from having to remember the bizon3 variant.
DEFAULT_PARAMS_FILES = {
    'bizon3': 'bizon_behavior_params_bizon3.yaml',
}
DEFAULT_PARAMS_FILE = 'bizon_behavior_params.yaml'


def launch_setup(context, *args, **kwargs):
    bringup_dir = get_package_share_directory('bizon_player_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    prefix = LaunchConfiguration('prefix').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')
    verbose = LaunchConfiguration('verbose').perform(context)
    move_group_log_level = LaunchConfiguration('move_group_log_level').perform(context)
    moveit_config_package = LaunchConfiguration('moveit_config_package')

    params_file = LaunchConfiguration('params_file').perform(context)
    if not params_file:
        params_file = os.path.join(
            bringup_dir, 'params',
            DEFAULT_PARAMS_FILES.get(prefix, DEFAULT_PARAMS_FILE),
        )

    lifecycle_delay = float(LaunchConfiguration('lifecycle_delay').perform(context))
    client_delay = float(LaunchConfiguration('client_delay').perform(context))

    # MoveIt, ros2_control and robot_state_publisher. Isaac Sim must already be
    # running: ros2_control talks to its joint interfaces.
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bizon_player_bringup.launch.py')),
        launch_arguments={
            'prefix': prefix,
            'use_sim_time': use_sim_time,
            'moveit_config_package': moveit_config_package,
            'verbose': verbose,
            'move_group_log_level': move_group_log_level,
        }.items(),
    )

    # behavior_server plus the lifecycle manager that brings it up. The
    # underlying launch file calls this same robot 'namespace'.
    lifecycle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bizon_lifecycle_dev.launch.py')),
        launch_arguments={
            'namespace': prefix,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'log_level': log_level,
            'params_file': params_file,
        }.items(),
    )

    # Same invocation as the README's `ros2 run` line. The behavior parameters
    # are deliberately not passed here: the client is started bare today, and
    # handing it params_file would change which side it plays.
    behavior_tree_client = Node(
        package='bizon_behavior_clients',
        executable='bizon_behavior_tree_client_main',
        name='bizon_behavior_tree',
        namespace='/' + prefix,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
        emulate_tty=True,
    )

    # Both delays are measured from launch start, so client_delay must stay
    # above lifecycle_delay.
    return [
        bringup,
        TimerAction(period=lifecycle_delay, actions=[lifecycle]),
        TimerAction(period=client_delay, actions=[behavior_tree_client]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'prefix',
            default_value='bizon2',
            description='Robot name, used as both the bringup prefix and the ROS namespace'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the simulation clock'),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Let the lifecycle manager activate behavior_server on startup'),
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Put the infrastructure nodes back on screen instead of the log files'),
        DeclareLaunchArgument(
            'move_group_log_level',
            default_value='warn',
            description="move_group's own log level"),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for behavior_server, the lifecycle manager and the tree client'),
        DeclareLaunchArgument(
            'moveit_config_package',
            default_value='bizon2_moveit_pkg',
            description='MoveIt config package'),
        DeclareLaunchArgument(
            'params_file',
            default_value='',
            description='Behavior parameters. Empty selects the file matching prefix'),
        DeclareLaunchArgument(
            'lifecycle_delay',
            default_value='10.0',
            description='Seconds to wait for move_group before starting behavior_server'),
        DeclareLaunchArgument(
            'client_delay',
            default_value='15.0',
            description='Seconds to wait before starting the behavior tree client. '
                        'Measured from launch start, so keep it above lifecycle_delay'),
    ] + [OpaqueFunction(function=launch_setup)])
