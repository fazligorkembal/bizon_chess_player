export isaac_sim_package_path=$HOME/Documents/isaac-sim-standalone-5.1.0

export ROS_DISTRO=humble

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Can only be set once per terminal.
# Setting this command multiple times will append the internal library path again potentially leading to conflicts
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/humble/lib

# Run Isaac Sim
#$isaac_sim_package_path/python.sh $HOME/Documents/projects/bizon_chess_player/isaac_sim_standalone/chessboard_image_generator.py
$isaac_sim_package_path/python.sh $HOME/Documents/projects/bizon_chess_player/isaac_sim_standalone/chessboard_fen_to_board.py
#$isaac_sim_package_path/python.sh /home/gorkem/Documents/isaac-sim-standalone-5.1.0/standalone_examples/api/isaacsim.ros2.bridge/rtx_lidar.py