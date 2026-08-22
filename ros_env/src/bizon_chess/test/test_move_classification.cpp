#include <gtest/gtest.h>

#include <string>

#include "bizon_chess/move_classification.hpp"

// classifyMoveType() decides which of six very different pick-and-place
// routines DecisionPlugin runs for a move (straight/capture/en_passant/
// castle/promotion/promotion_capture), so a misclassification sends the arm
// through the wrong sequence entirely. These FENs are handcrafted, minimal
// boards -- just the pieces each branch's scan actually looks at -- rather
// than realistic games, so each test isolates exactly one branch.

using bizon_chess::classifyMoveType;

TEST(MoveClassification, PawnPushWithNothingOnTheTargetSquareIsStraight)
{
  EXPECT_EQ(
    classifyMoveType("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1", "e2e4"),
    "straight");
}

TEST(MoveClassification, MoveOntoAnOccupiedSquareIsCapture)
{
  // Only the two pawns and the two kings matter: white e4 takes black d5.
  EXPECT_EQ(classifyMoveType("8/8/8/3p4/4P3/8/8/4K2k w - - 0 1", "e4d5"), "capture");
}

TEST(MoveClassification, MoveOntoTheEnPassantSquareIsEnPassant)
{
  // Black just played d7-d5; white's e5 pawn may capture en passant on the
  // empty d6 square, which is why d6 (not d5, where the captured pawn
  // actually sits) is what makes this branch fire.
  EXPECT_EQ(classifyMoveType("8/8/8/3pP3/8/8/8/4K2k w - d6 0 1", "e5d6"), "en_passant");
}

TEST(MoveClassification, FiveCharMoveOntoAnEmptySquareIsPromotion)
{
  EXPECT_EQ(classifyMoveType("8/P7/8/8/8/8/8/4K2k w - - 0 1", "a7a8q"), "promotion");
}

TEST(MoveClassification, FiveCharMoveOntoAnOccupiedSquareIsPromotionCapture)
{
  EXPECT_EQ(classifyMoveType("n7/P7/8/8/8/8/8/4K2k w - - 0 1", "a7a8q"), "promotion_capture");
}

TEST(MoveClassification, KingE1ToG1WithKingsideRightsIsShortCastle)
{
  EXPECT_EQ(
    classifyMoveType("r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1", "e1g1"),
    "short_castle");
}

TEST(MoveClassification, KingE1ToC1WithQueensideRightsIsLongCastle)
{
  EXPECT_EQ(
    classifyMoveType("r3k2r/8/8/8/8/8/8/R3K2R w KQkq - 0 1", "e1c1"),
    "long_castle");
}

TEST(MoveClassification, BlackKingE8ToG8WithKingsideRightsIsShortCastle)
{
  EXPECT_EQ(
    classifyMoveType("r3k2r/8/8/8/8/8/8/R3K2R b KQkq - 0 1", "e8g8"),
    "short_castle");
}
