#ifndef BIZON_CHESS__ROBOT_PARAMS_HPP_
#define BIZON_CHESS__ROBOT_PARAMS_HPP_

namespace bizon_chess
{
/// Everything that was previously a compile-time constant in
/// make_decision_client_node.hpp:103-110. Populated from ROS parameters at
/// runtime so the values can be replaced by calibration on real hardware.
struct RobotParams
{
  double box_size = 0.03718857142;   ///< edge length of one board square [m]
  double robot_base_offset_x = -0.3; ///< arm base X in the board frame [m]
  double link_l1 = 0.29;             ///< proximal link length [m]
  double link_l2 = 0.18;             ///< distal link length [m]
  double gap_eef_close = 0.04;       ///< gripper closed finger gap [m]
  double gap_eef_open = 0.20;        ///< gripper open finger gap [m]
  double limit_l1_down = 0.155;      ///< prismatic travel when placing [m]
  double limit_l1_up = 0.08;         ///< prismatic travel when clear [m]
  bool mirrored = false;             ///< true for the black-side robot
};
}  // namespace bizon_chess
#endif  // BIZON_CHESS__ROBOT_PARAMS_HPP_
