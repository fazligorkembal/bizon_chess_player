#include <gtest/gtest.h>
#include <chrono>
#include <string>

#include "bizon_behavior_servers/stockfish_process.hpp"

using bizon_behaviors::StockfishProcess;

// Requires the `stockfish` binary on PATH, as the README's install step provides.
TEST(StockfishProcess, StartsAndHandshakes)
{
  StockfishProcess sf;
  ASSERT_TRUE(sf.start(std::chrono::seconds(5))) << sf.lastError();
  EXPECT_TRUE(sf.isRunning());
  sf.stop();
  EXPECT_FALSE(sf.isRunning());
}

TEST(StockfishProcess, ReturnsBestMoveForOpeningPosition)
{
  StockfishProcess sf;
  ASSERT_TRUE(sf.start(std::chrono::seconds(5))) << sf.lastError();

  std::string best;
  ASSERT_TRUE(sf.bestMove(
    "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
    10, std::chrono::seconds(10), best)) << sf.lastError();

  // A legal long-algebraic move: two squares, optionally a promotion char.
  ASSERT_GE(best.size(), 4u);
  EXPECT_GE(best[0], 'a'); EXPECT_LE(best[0], 'h');
  EXPECT_GE(best[1], '1'); EXPECT_LE(best[1], '8');
  EXPECT_GE(best[2], 'a'); EXPECT_LE(best[2], 'h');
  EXPECT_GE(best[3], '1'); EXPECT_LE(best[3], '8');
  sf.stop();
}

// The bug this class exists to fix: a hung engine must not block forever.
TEST(StockfishProcess, TimesOutInsteadOfBlockingForever)
{
  StockfishProcess sf;
  ASSERT_TRUE(sf.start(std::chrono::seconds(5))) << sf.lastError();

  std::string best;
  const auto t0 = std::chrono::steady_clock::now();
  // A zero-length deadline can never be met, so this must return false fast.
  const bool ok = sf.bestMove(
    "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
    30, std::chrono::milliseconds(1), best);
  const auto elapsed = std::chrono::steady_clock::now() - t0;

  EXPECT_FALSE(ok);
  EXPECT_LT(std::chrono::duration_cast<std::chrono::seconds>(elapsed).count(), 3);
  sf.stop();
}

TEST(StockfishProcess, ReportsFailureForMissingBinary)
{
  StockfishProcess sf("definitely-not-a-real-engine-binary");
  EXPECT_FALSE(sf.start(std::chrono::seconds(2)));
  EXPECT_FALSE(sf.lastError().empty());
}
