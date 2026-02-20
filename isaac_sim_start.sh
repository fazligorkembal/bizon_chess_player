export isaac_sim_package_path=$HOME/Documents/isaac-sim-standalone-5.1.0

export ROS_DISTRO=humble

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Can only be set once per terminal.
# Setting this command multiple times will append the internal library path again potentially leading to conflicts
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$isaac_sim_package_path/exts/isaacsim.ros2.bridge/humble/lib

# Run Isaac Sim
$isaac_sim_package_path/isaac-sim.sh