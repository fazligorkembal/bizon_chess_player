// Unit tests for the ROS-free formatting helpers behind the debug logging
// system: event-line formatting, bundle directory naming, cell/square
// mapping, and the FEN square-level diff. None of this needs a ROS graph --
// see debug_session_format.hpp's header comment.
#include <gtest/gtest.h>

#include <chrono>
#include <ctime>

#include "bizon_behavior_servers/debug_session_format.hpp"

namespace
{
// 2026-08-23 15:43:02.114 local time, used across the tests below so the
// expected strings are simple literals instead of recomputed at test time.
std::chrono::system_clock::time_point fixedTime()
{
  std::tm tm_buf{};
  tm_buf.tm_year = 2026 - 1900;
  tm_buf.tm_mon = 8 - 1;
  tm_buf.tm_mday = 23;
  tm_buf.tm_hour = 15;
  tm_buf.tm_min = 43;
  tm_buf.tm_sec = 2;
  tm_buf.tm_isdst = -1;
  const std::time_t t = std::mktime(&tm_buf);
  return std::chrono::system_clock::from_time_t(t) + std::chrono::milliseconds(114);
}
}  // namespace

namespace bizon_behaviors::debug_format
{

TEST(DebugSessionFormat, TimestampShapeAndMillis)
{
  const std::string ts = formatTimestamp(fixedTime());
  EXPECT_EQ(ts, "2026-08-23 15:43:02.114");
}

TEST(DebugSessionFormat, LogLineShape)
{
  const std::string line = formatLogLine(fixedTime(), "board", "fen=8/8/8/8/8/8/8/8");
  EXPECT_EQ(line, "2026-08-23 15:43:02.114 | board | fen=8/8/8/8/8/8/8/8");
}

TEST(DebugSessionFormat, BundleDirNamePadsCounterAndUsesTimeOfDay)
{
  EXPECT_EQ(formatBundleDirName(3, "fen_mismatch", fixedTime()), "003_fen_mismatch_15-43-02");
  EXPECT_EQ(formatBundleDirName(42, "board_detect_failed", fixedTime()), "042_board_detect_failed_15-43-02");
}

TEST(DebugSessionFormat, BundleDirNameCounterGrowsPastThreeDigitsInsteadOfBreaking)
{
  EXPECT_EQ(formatBundleDirName(1000, "no_move_owner", fixedTime()), "1000_no_move_owner_15-43-02");
}

TEST(DebugSessionFormat, SquareForCellIndexWhiteSideIsIdentityOrder)
{
  // index 0 == a8 (top-left of a standard FEN board field), index 63 == h1.
  EXPECT_EQ(squareForCellIndex(0, /*black_side=*/false), "a8");
  EXPECT_EQ(squareForCellIndex(7, /*black_side=*/false), "h8");
  EXPECT_EQ(squareForCellIndex(56, /*black_side=*/false), "a1");
  EXPECT_EQ(squareForCellIndex(63, /*black_side=*/false), "h1");
  // one arbitrary interior square: index 28 -> rank_from_top 3, file 4 -> e5
  EXPECT_EQ(squareForCellIndex(28, /*black_side=*/false), "e5");
}

TEST(DebugSessionFormat, SquareForCellIndexBlackSideIsReversed)
{
  // Mirrors BoardPlugin::cameraCallback's std::reverse() of class_names for
  // the black-side robot: cell_images_[0] on that robot ends up labelling
  // board index 63 (h1), not a8.
  EXPECT_EQ(squareForCellIndex(0, /*black_side=*/true), "h1");
  EXPECT_EQ(squareForCellIndex(63, /*black_side=*/true), "a8");
}

TEST(DebugSessionFormat, FenSquareDiffEmptyWhenIdentical)
{
  const std::string fen = "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR";
  EXPECT_TRUE(fenSquareDiff(fen, fen).empty());
}

TEST(DebugSessionFormat, FenSquareDiffReportsEachDisagreement)
{
  // Adapted from the bug report's camera FEN: two ranks disagree on the
  // back-rank-vs-empty question at a couple of squares.
  const std::string camera = "8/8/8/8/8/8/8/nqnn1qqr";
  const std::string text = "8/8/8/8/8/8/8/RNBQKBNR";
  const auto diffs = fenSquareDiff(camera, text);
  // Every one of the 8 back-rank squares disagrees (different piece or
  // colour at each), so the diff should have exactly 8 lines, each naming a
  // rank-1 square.
  EXPECT_EQ(diffs.size(), 8u);
  for (const auto & line : diffs) {
    EXPECT_NE(line.find("1: camera="), std::string::npos) << line;
    EXPECT_NE(line.find(" text="), std::string::npos) << line;
  }
}

TEST(DebugSessionFormat, FenSquareDiffDescribesEmptyExplicitly)
{
  const std::string camera = "8/8/8/8/8/8/8/8";        // fully empty
  const std::string text = "8/8/8/8/8/8/8/7Q";          // one queen on h1
  const auto diffs = fenSquareDiff(camera, text);
  ASSERT_EQ(diffs.size(), 1u);
  EXPECT_EQ(diffs[0], "h1: camera=empty text=Q");
}

}  // namespace bizon_behaviors::debug_format
