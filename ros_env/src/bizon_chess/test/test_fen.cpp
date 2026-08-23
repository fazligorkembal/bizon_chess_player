#include <gtest/gtest.h>
#include <algorithm>
#include <string>
#include <vector>

#include "bizon_chess/fen.hpp"

using bizon_chess::occupiedSquares;

namespace {
bool contains(const std::vector<std::string> & v, const std::string & s)
{
  return std::find(v.begin(), v.end(), s) != v.end();
}
}  // namespace

TEST(Fen, OpeningPositionHasThirtyTwoOccupiedSquares)
{
  const auto squares =
    occupiedSquares("rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1");
  EXPECT_EQ(squares.size(), 32u);
  EXPECT_TRUE(contains(squares, "a1"));
  EXPECT_TRUE(contains(squares, "e8"));
  EXPECT_TRUE(contains(squares, "h2"));
  EXPECT_FALSE(contains(squares, "e4"));
}

TEST(Fen, EmptyBoardHasNoOccupiedSquares)
{
  EXPECT_TRUE(occupiedSquares("8/8/8/8/8/8/8/8 w - - 0 1").empty());
}

TEST(Fen, SinglePieceIsPlacedOnTheCorrectSquare)
{
  // Rank 8 is the first field; four empty files then a white king on e8.
  const auto squares = occupiedSquares("4K3/8/8/8/8/8/8/8 w - - 0 1");
  ASSERT_EQ(squares.size(), 1u);
  EXPECT_EQ(squares[0], "e8");
}

TEST(Fen, HandlesBoardOnlyFenWithoutTrailingFields)
{
  const auto squares = occupiedSquares("8/8/8/4p3/8/8/8/8");
  ASSERT_EQ(squares.size(), 1u);
  EXPECT_EQ(squares[0], "e5");
}

TEST(Fen, ReturnsEmptyForMalformedInput)
{
  EXPECT_TRUE(occupiedSquares("").empty());
  EXPECT_TRUE(occupiedSquares("not-a-fen").empty());
}
