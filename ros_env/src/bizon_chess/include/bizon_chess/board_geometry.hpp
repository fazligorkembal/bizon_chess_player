#ifndef BIZON_CHESS__BOARD_GEOMETRY_HPP_
#define BIZON_CHESS__BOARD_GEOMETRY_HPP_

#include <string>
#include "bizon_chess/robot_params.hpp"

namespace bizon_chess
{
/// Convert an algebraic square ("e4") to board-frame XY in metres.
/// Returns false if the square is malformed.
bool squareToWorld(const std::string & square, const RobotParams & p, double & x, double & y);

/// Re-expresses a board-frame XY in the acting robot's own base frame.
/// White sits at the board frame's origin side, so only the arm's base
/// offset needs subtracting from X. Black's arm is the mirror image of
/// white's across both axes (it is bolted to the opposite edge of the same
/// board), so X is negated before the same offset is subtracted and Y is
/// negated outright. Selected by RobotParams::mirrored so this one function
/// is the single place either convention is applied -- see the ADR entry
/// this task closes (F6) for why a second, drifting copy of this arithmetic
/// in a robot-specific plugin was worse than no planning scene at all.
void mirrorForRobotSide(const RobotParams & p, double & x, double & y);

/// squareToWorld() followed by mirrorForRobotSide(): the composition every
/// caller that places a piece or plans a motion actually wants. Returns
/// false if the square is malformed.
bool squareToWorldMirrored(const std::string & square, const RobotParams & p, double & x, double & y);

/// Two-link planar inverse kinematics, elbow-up solution.
/// Returns false if the target is outside the annulus the arm can reach.
bool worldToJointAngles(double x, double y, const RobotParams & p, double & q1, double & q2);
}  // namespace bizon_chess
#endif  // BIZON_CHESS__BOARD_GEOMETRY_HPP_
