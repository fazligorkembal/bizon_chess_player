#include "bizon_chess/board_geometry.hpp"

#include <cmath>

namespace bizon_chess
{

bool squareToWorld(const std::string & square, const RobotParams & p, double & x, double & y)
{
  if (square.size() != 2) {
    return false;
  }
  const char file_c = square[0];
  const char rank_c = square[1];
  if (file_c < 'a' || file_c > 'h' || rank_c < '1' || rank_c > '8') {
    return false;
  }

  const int col = file_c - 'a';  // 0..7
  const int row = rank_c - '1';  // 0..7

  // Same convention as the original get_box_location_wrt_world():
  // rank runs along +X, file runs along -Y, board centred on the origin.
  x = -3.5 * p.box_size + row * p.box_size;
  y = 3.5 * p.box_size - col * p.box_size;
  return true;
}

bool worldToJointAngles(double x, double y, const RobotParams & p, double & q1, double & q2)
{
  const double r2 = x * x + y * y;
  const double denom = 2.0 * p.link_l1 * p.link_l2;
  if (denom == 0.0) {
    return false;
  }

  const double d = (r2 - p.link_l1 * p.link_l1 - p.link_l2 * p.link_l2) / denom;
  // |d| > 1 means the target is outside the reachable annulus. The original
  // code fed this straight into sqrtf(1 - d*d) and produced NaN joint targets.
  if (!(d >= -1.0 && d <= 1.0)) {
    return false;
  }

  q2 = std::atan2(std::sqrt(1.0 - d * d), d);
  q1 = std::atan2(y, x) - std::atan2(p.link_l2 * std::sin(q2), p.link_l1 + p.link_l2 * std::cos(q2));
  return true;
}

}  // namespace bizon_chess
