#include <gtest/gtest.h>
#include <cmath>
#include "bizon_chess/board_geometry.hpp"

using bizon_chess::RobotParams;
using bizon_chess::squareToWorld;
using bizon_chess::worldToJointAngles;

namespace {
RobotParams defaultParams()
{
  RobotParams p;
  p.box_size = 0.03718857142;
  p.robot_base_offset_x = -0.3;
  p.link_l1 = 0.29;
  p.link_l2 = 0.18;
  return p;
}
}  // namespace

// The board is centred on the world origin, so the four centre squares
// straddle it symmetrically at +/- half a square.
TEST(BoardGeometry, CentreSquaresStraddleOrigin)
{
  const auto p = defaultParams();
  double xd4, yd4, xe5, ye5;
  ASSERT_TRUE(squareToWorld("d4", p, xd4, yd4));
  ASSERT_TRUE(squareToWorld("e5", p, xe5, ye5));
  EXPECT_NEAR(xd4, -0.5 * p.box_size, 1e-9);
  EXPECT_NEAR(xe5, 0.5 * p.box_size, 1e-9);
  EXPECT_NEAR(yd4, 0.5 * p.box_size, 1e-9);
  EXPECT_NEAR(ye5, -0.5 * p.box_size, 1e-9);
}

// Corners must sit 3.5 squares from the centre on both axes.
TEST(BoardGeometry, CornersAreSevenHalfSquaresApart)
{
  const auto p = defaultParams();
  double xa1, ya1, xh8, yh8;
  ASSERT_TRUE(squareToWorld("a1", p, xa1, ya1));
  ASSERT_TRUE(squareToWorld("h8", p, xh8, yh8));
  EXPECT_NEAR(xh8 - xa1, 7.0 * p.box_size, 1e-9);
  EXPECT_NEAR(ya1 - yh8, 7.0 * p.box_size, 1e-9);
}

TEST(BoardGeometry, RejectsMalformedSquare)
{
  const auto p = defaultParams();
  double x, y;
  EXPECT_FALSE(squareToWorld("", p, x, y));
  EXPECT_FALSE(squareToWorld("j1", p, x, y));
  EXPECT_FALSE(squareToWorld("a9", p, x, y));
  EXPECT_FALSE(squareToWorld("a", p, x, y));
}

// Forward kinematics of the 2-link solution must land back on the request.
TEST(BoardGeometry, JointAnglesReproduceTargetPosition)
{
  const auto p = defaultParams();
  const double x = 0.30;
  const double y = 0.05;
  double q1, q2;
  ASSERT_TRUE(worldToJointAngles(x, y, p, q1, q2));

  const double fx = p.link_l1 * std::cos(q1) + p.link_l2 * std::cos(q1 + q2);
  const double fy = p.link_l1 * std::sin(q1) + p.link_l2 * std::sin(q1 + q2);
  EXPECT_NEAR(fx, x, 1e-6);
  EXPECT_NEAR(fy, y, 1e-6);
}

// Out of reach must be reported, not silently produce NaN.
TEST(BoardGeometry, RejectsUnreachableTarget)
{
  const auto p = defaultParams();
  double q1, q2;
  EXPECT_FALSE(worldToJointAngles(10.0, 10.0, p, q1, q2));
  EXPECT_FALSE(worldToJointAngles(0.0, 0.0, p, q1, q2));
}
