from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent, RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    behavior_server_node = LifecycleNode(
        package='bizon_behavior_servers',
        executable='behavior_server',
        name='behavior_server',
        namespace='bizon1',
        output='screen',
        parameters=[{
            'cycle_frequency': 20.0,
        }]
    )

    # Configure the behavior server
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node_name: node_name == '/behavior_server',
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # Activate the behavior server after it has been configured
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=behavior_server_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node_name: node_name == '/behavior_server',
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        behavior_server_node,
        configure_event,
        activate_event,
    ])
