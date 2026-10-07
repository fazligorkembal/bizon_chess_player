import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction


class Colors:
    black = '\033[30m'
    red = '\033[31m'
    green = '\033[32m'
    orange = '\033[33m'
    blue = '\033[34m'
    purple = '\033[35m'
    cyan = '\033[36m'
    lightgrey = '\033[37m'
    darkgrey = '\033[90m'
    lightred = '\033[91m'
    lightgreen = '\033[92m'
    yellow = '\033[93m'
    lightblue = '\033[94m'
    pink = '\033[95m'
    lightcyan = '\033[96m'
    end = '\033[0m'


class Moveit_config_bringer:

    def __init__(self):
        self.type_list = []
        self.old_prefix = "bizon1"

    def change_prefix(self, moveit_config: dict, prefix: str, old_prefix: str = "bizon1"):
        self.old_prefix = old_prefix

        if prefix == self.old_prefix:
            return moveit_config

        moveit_config = self.change_dict_prefix(moveit_config, prefix)

        return moveit_config

    def change_dict_prefix(self, dictionary: dict, prefix: str):

        keys = list(dictionary.keys())

        for key in keys:

            if self.old_prefix in key:
                new_key = key.replace(self.old_prefix, prefix)
                dictionary[new_key] = dictionary[key]
                del dictionary[key]
                key = new_key

            if type(dictionary[key]) == dict:
                dictionary[key] = self.change_dict_prefix(dictionary[key], prefix)
            elif type(dictionary[key]) == str:
                dictionary[key] = self.change_str_prefix(dictionary[key], prefix)
            elif type(dictionary[key]) == list:
                dictionary[key] = self.change_list_prefix(dictionary[key], prefix)
        return dictionary

    def change_list_prefix(self, list_, prefix):
        for i in range(len(list_)):
            if type(list_[i]) == dict:
                list_[i] = self.change_dict_prefix(list_[i], prefix)
            elif type(list_[i]) == str:
                list_[i] = self.change_str_prefix(list_[i], prefix)
            elif type(list_[i]) == list:
                list_[i] = self.change_list_prefix(list_[i], prefix)
        return list_

    def change_str_prefix(self, string, prefix):
        string = string.replace(self.old_prefix, prefix)
        return string


def launch_setup(context, *args, **kwargs):
    moveit_config_package = LaunchConfiguration('moveit_config_package')
    bringup_package = LaunchConfiguration('bringup_package')
    use_sim_time = LaunchConfiguration('use_sim_time')
    prefix = LaunchConfiguration('prefix')
    namespace = "/" + prefix.perform(context)
    ros2_control_hardware_type = LaunchConfiguration('ros2_control_hardware_type')
    use_controller = LaunchConfiguration('use_controller')

    print(Colors.yellow + "MoveIt config package: "
          + moveit_config_package.perform(context) + Colors.end)
    print(Colors.yellow + "Bringup package: " + bringup_package.perform(context) + Colors.end)
    print(Colors.yellow + "Use sim time: " + use_sim_time.perform(context) + Colors.end)
    print(Colors.yellow + "Robot name: " + prefix.perform(context) + Colors.end)
    print(Colors.yellow + "Namespace: " + namespace + Colors.end)
    print(Colors.yellow + "Ros2 control hardware type: "
          + ros2_control_hardware_type.perform(context) + Colors.end)
    print(Colors.yellow + "Use controller: " + use_controller.perform(context) + Colors.end)

    moveit_config = (
        MoveItConfigsBuilder("bizon", package_name=moveit_config_package.perform(context))
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    moveit_config_bringer = Moveit_config_bringer()
    moveit_config = moveit_config_bringer.change_prefix(
        moveit_config.to_dict(), prefix.perform(context))

    ros2_controllers_yaml = os.path.join(
        get_package_share_directory(bringup_package.perform(context)), "params",
        prefix.perform(context) + "_ros2_controllers" + ".yaml"
    )

    if(os.path.exists(ros2_controllers_yaml)):
        print(Colors.green + "Controllers file found: " + str(ros2_controllers_yaml) + Colors.end)
    else:
        print(Colors.red + "Controllers file not found: "
              + str(ros2_controllers_yaml) + Colors.end)

    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }

    robot_description = {"robot_description": moveit_config['robot_description']}

    move_group_params = [
        moveit_config,
        move_group_configuration,
        {'use_sim_time': use_sim_time},
    ]

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        arguments=[
            "--ros-args",
            "--log-level",
            "move_group:=DEBUG",
            ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )

    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_description, ros2_controllers_yaml, {'use_sim_time': use_sim_time}],
    #     output="both",
    #     arguments=["--ros-args", "--log-level", "info"],
    # )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[
            robot_description,
            ros2_controllers_yaml,
            {"use_sim_time": use_sim_time},
        ],
    )

    load_controllers = []
    for controller in ["joint_state_broadcaster", "arm_group_controller"]:
        load_controllers.append(
            ExecuteProcess(
                cmd=[
                    f"ros2 run controller_manager spawner {controller} "
                    f"-c /{prefix.perform(context)}/controller_manager"
                ],
                shell=True,
                output="screen",
            )
        )

    n = GroupAction(
        actions=[
            # rviz_node,
            PushRosNamespace(prefix.perform(context)),
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
        ]
        + load_controllers
    )

    return [n]


def generate_launch_description():

    declare_moveit_config_package = DeclareLaunchArgument(
        'moveit_config_package',
        default_value='bizon2_isaac_moveit_pkg',
        description='MoveIt config package')

    declare_bringup_package = DeclareLaunchArgument(
        'bringup_package',
        default_value='bizon_player_bringup',
        description='Description package')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='None')

    declare_prefix = DeclareLaunchArgument(
        'prefix',
        default_value='bizon2',
        description='Robot name')

    declare_ros2_control_hardware_type = DeclareLaunchArgument(
        'ros2_control_hardware_type',
        default_value='isaac',
        description='None',
        choices=['rviz', 'real', 'isaac']
    )

    declare_use_controller = DeclareLaunchArgument(
        'use_controller',
        default_value='False',
        description='None')

    return LaunchDescription([
            declare_moveit_config_package,
            declare_bringup_package,
            declare_use_sim_time,
            declare_prefix,
            declare_ros2_control_hardware_type,
            declare_use_controller,
        ]
        + [OpaqueFunction(function=launch_setup)]
    )
