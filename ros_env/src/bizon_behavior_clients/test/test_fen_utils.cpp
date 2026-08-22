#include <gtest/gtest.h>

#include <string>

#include "bizon_behavior_clients/plugins/action/fen_utils.hpp"

// These helpers exist because who_is_owner_of_move() used bare ternaries to
// turn a FEN's side-to-move field into an owner name, and the two cases --
// "the FEN we matched" versus "the parent the match descends from" -- need
// opposite mappings. Nothing in the expression said which case it was, and a
// parent-mapping was applied to a matched FEN. That returned "black" for a
// position whose FEN says "w", the robot concluded it was the opponent's turn
// on its own move, and the tree eventually gave up. Named functions make the
// choice impossible to get wrong silently.

namespace
{
using bizon_behavior_clients::side_to_move;
using bizon_behavior_clients::side_to_move_after_one_ply;
}  // namespace

TEST(FenUtils, SideToMoveReadsTheFenField)
{
  EXPECT_EQ(side_to_move("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"), "white");
  EXPECT_EQ(side_to_move("rnbqkbnr/pppppppp/8/8/8/4P3/PPPP1PPP/RNBQKBNR b KQkq - 0 1"), "black");
}

TEST(FenUtils, SideToMoveAfterOnePlyFlipsTheFenField)
{
  EXPECT_EQ(
    side_to_move_after_one_ply("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1"),
    "black");
  EXPECT_EQ(
    side_to_move_after_one_ply("rnbqkbnr/pppppppp/8/8/8/4P3/PPPP1PPP/RNBQKBNR b KQkq - 0 1"),
    "white");
}

TEST(FenUtils, MalformedFenYieldsNoSide)
{
  EXPECT_EQ(side_to_move("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR"), "");
  EXPECT_EQ(side_to_move(""), "");
  EXPECT_EQ(side_to_move_after_one_ply("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR"), "");
}

// The exact position from the failing Isaac Sim run. white_last_moves.txt had
// walked to move 3 with white to move, the camera agreed, and the second-degree
// branch of who_is_owner_of_move() still reported "black (opponent). waiting".
TEST(FenUtils, MatchedPositionFromTheSecondDegreeSearchIsOwnedByTheSideItsFenNames)
{
  const std::string matched_camera_position =
    "rnbqkbnr/pppp1ppp/8/8/3p4/4P3/PPP2PPP/RNBQKBNR w KQkq - 0 3";
  const std::string its_parent =
    "rnbqkbnr/pppp1ppp/8/4p3/3P4/4P3/PPP2PPP/RNBQKBNR b KQkq - 0 2";

  // The position the camera sees is the one being played from: white moves.
  EXPECT_EQ(side_to_move(matched_camera_position), "white");

  // Its parent had black to move, so one ply later it is white's turn -- the
  // two routes must agree, which is what the buggy call site violated.
  EXPECT_EQ(side_to_move_after_one_ply(its_parent), "white");
  EXPECT_EQ(side_to_move_after_one_ply(its_parent), side_to_move(matched_camera_position));
}
