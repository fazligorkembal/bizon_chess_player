test moveit wo namespace
ros2 launch bizon_moveit_pkg demo.launch.py
ros2 run test_moveit_pg first_app

test isaac sim wo namespace
ros2 launch bizon2_isaac_moveit_pkg demo.launch.py

test isaac sim with namespace wo rviz
ros2 launch bizon_player_bringup test_bringup.launch.py + play isaac sim wrt to topics ... !

****FULL REMINDER****
1) ros2 run xacro xacro src/bizon_description/urdf/bizon.urdf.xacro > src/bizon_description/urdf/bizon2_full_isaac.urdf prefix:=bizon2

2) colcon build

3) ros2 run moveit_setup_assistant moveit_setup_assistant
    - remove ros2_control tag under bizon2_full_isaac.urdf and add them to bizon_moveit_pkg/config/bizon.ros2_control.xacro

    * Not change tractory setups for gripper
    
    * add action_ns: follow_joint_trajectory in moveit_controllers.yaml !!!!!!!!!!!!!!!!!!!!!!!! 
    Example
    '''
    # MoveIt uses this configuration for controller management

moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - arm_group_controller
    - hand_group_controller

  arm_group_controller:
    type: FollowJointTrajectory
    joints:
      - bizon2rev1
      - bizon2pris1
      - bizon2rev2
      - bizon2fixed_gripper_base
    action_ns: follow_joint_trajectory
    default: true
  hand_group_controller:
    type: FollowJointTrajectory
    joints:
      - bizon2gripper_finger1_joint
      - bizon2gripper_finger2_joint
      - bizon2gripper_finger3_joint
    action_ns: follow_joint_trajectory
    default: true
   '''


4) colcon build && . install/setup.bash

*** BASIC MOVE TEST
1) TEST - ros2 launch bizon_player_bringup bizon_player_bringup.launch.py
2) ros2 run test_moveit_pg first_app --ros-args -r __ns:=/bizon2 -p use_sim_time:=true

****LIFECYCLE MANAGER TEST****
1) ros2 launch bizon_player_bringup bizon_lifecycle_dev.launch.py use_sim_time:=false (wo sim)




* ISAAC SIM *
set isaac_sim_package_path=C:\Users\Gorkem\Desktop\isaac_sim_5_1
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
set PATH=%PATH%;%isaac_sim_package_path%\exts\isaacsim.ros2.bridge\humble\lib
%isaac_sim_package_path%\isaac-sim.bat --/isaac/startup/ros_bridge_extension=isaacsim.ros2.bridge

* ISAAC SIM STANDALONE
set isaac_sim_package_path=C:\Users\Gorkem\Desktop\isaac_sim_5_1
set RMW_IMPLEMENTATION=rmw_fastrtps_cpp
set PATH=%PATH%;%isaac_sim_package_path%\exts\isaacsim.ros2.bridge\humble\lib
%isaac_sim_package_path%\python.bat %isaac_sim_package_path%\standalone_examples\api\isaacsim.ros2.bridge\clock.py


# Robot ekleme
xacrodan urdfe cevir. prefix:=robotname
moveit doldur
moveit_controllers.yaml icine her group icin 
  * action_ns: follow_joint_trajectory
  * default: true
urdf icindeki ros2_control tagini bizon.ros2_contrl.xacro altina tasi urdften sil.


export FASTRTPS_DEFAULT_PROFILES_FILE=


ros2 launch bizon_player_bringup bizon_player_bringup.launch.py prefix:=bizon2
ros2 launch bizon_player_bringup bizon_lifecycle_dev.launch.py namespace:=bizon2
ros2 run bizon_behavior_clients bizon_behavior_tree_client_main --ros-args -r __ns:=/bizon2 -p use_sim_time:=true 2>&1 | sed   -e '/\[ERROR\]/{s/.*/\x1b[31m&\x1b[0m/;b}'   -e '/\[WARN\]/{s/.*/\x1b[33m&\x1b[0m/;b}'   -e 's/\(.*bizon_behavior_tree.*\)/\x1b[32m\1\x1b[0m/'   -e 's/\(.*MakeDecisionNode.*\)/\x1b[36m\1\x1b[0m/'   -e 's/\(.*BoardActionIsaacClientNode.*\)/\x1b[35m\1\x1b[0m/'   -e 's/\(.*ArmActionClientNode.*\)/\x1b[35m\1\x1b[0m/'   -e 's/\(.*BtActionClientNode.*\)/\x1b[34m\1\x1b[0m/'



////////////////////////////////////////
Isaac Sim
isaac_sim_start.sh -> for open automatically isaac sim
isaac_sim_standalone -> open isaac sim with usd. (change .py path in the .sh to different scenerios such as chessboard.py or chessboard_datageneretor.py)

////////////////////////////////////////
display isaac annotation
python3 test_isaac_annotation/test_isaac_annotation.py



////////////////////////////////////////
LAUNCH & PARAM FILES (test_moveit_pg test_config.launch.py && test_param_executable.cpp)