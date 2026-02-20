from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Hangi MoveIt config paketini kullanacağını seç
    # bizon2_full_isaac_moveit_config veya bizon2_isaac_moveit_pkg
    moveit_config = MoveItConfigsBuilder(
        "bizon",
        package_name="bizon2_full_isaac_moveit_config"
    ).to_moveit_configs()

    # Your application node with all MoveIt parameters loaded
    first_app_node = Node(
        package="test_moveit_pg",
        executable="first_app",
        name="first_app_node",
        namespace="bizon2",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    return LaunchDescription([first_app_node])
