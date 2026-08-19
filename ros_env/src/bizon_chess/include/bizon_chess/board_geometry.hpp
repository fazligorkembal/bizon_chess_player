#ifndef BIZON_CHESS__BOARD_GEOMETRY_HPP_
#define BIZON_CHESS__BOARD_GEOMETRY_HPP_

#include <string>
#include "bizon_chess/robot_params.hpp"

namespace bizon_chess
{
/// Convert an algebraic square ("e4") to board-frame XY in metres.
/// Returns false if the square is malformed.
bool squareToWorld(const std::string & square, const RobotParams & p, double & x, double & y);

/// Two-link planar inverse kinematics, elbow-up solution.
/// Returns false if the target is outside the annulus the arm can reach.
bool worldToJointAngles(double x, double y, const RobotParams & p, double & q1, double & q2);
}  // namespace bizon_chess
#endif  // BIZON_CHESS__BOARD_GEOMETRY_HPP_
