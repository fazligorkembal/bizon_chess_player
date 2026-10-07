import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


class Colors:
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    END = '\033[0m'


def launch_setup(context, *args, **kwargs):

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    namespace = LaunchConfiguration('namespace')
    namespace2 = LaunchConfiguration('namespace2')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    params_file = LaunchConfiguration('params_file')

    # if not os.path.isfile(params_file.perform(context)):
    #     print(Colors.RED + 'Error: params_file does not exist: '
    #           + params_file.perform(context) + Colors.END)
    #     return []

    print(Colors.CYAN + 'Launching with namespace1: ' + namespace.perform(context) + Colors.END)
    print(Colors.CYAN + 'Launching with namespace2: ' + namespace2.perform(context) + Colors.END)
    print(Colors.CYAN + 'Launching with log_level: ' + log_level.perform(context) + Colors.END)
    print(Colors.CYAN + 'Launching with use_sim_time: '
          + use_sim_time.perform(context) + Colors.END)
    print(Colors.CYAN + 'Launching with remappings: ' + str(remappings) + Colors.END)
    print(Colors.CYAN + 'Launching with param_file: ' + params_file.perform(context) + Colors.END)

    test_param_node = GroupAction(
        actions=[
            PushRosNamespace(namespace),
            Node(
                package='test_moveit_pg',
                executable='test_param_executable',
                name='test_param_executable',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[params_file],
                remappings=remappings,
            ),
        ]
    )

    test_param_node2 = GroupAction(
        actions=[
            PushRosNamespace(namespace2),
            Node(
                package='test_moveit_pg',
                executable='test_param_executable',
                name='test_param_executable',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[params_file],
                remappings=remappings,
            ),
        ]
    )

    return [test_param_node, test_param_node2]


def generate_launch_description():
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='bizon1', description='Top-level namespace'
    )

    declare_namespace2_cmd = DeclareLaunchArgument(
        'namespace2', default_value='bizon2', description='Top-level namespace for second node'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack',
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    config_dir = get_package_share_directory('test_moveit_pg')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(config_dir, 'params', 'test_param_executable.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    return LaunchDescription(
        [
            declare_namespace_cmd,
            declare_namespace2_cmd,
            declare_use_sim_time_cmd,
            declare_autostart_cmd,
            declare_log_level_cmd,
            declare_params_file_cmd,
        ] + [OpaqueFunction(function=launch_setup)]
    )
