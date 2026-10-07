#include <gtest/gtest.h>
#include <chrono>
#include <string>
#include <vector>

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
  ASSERT_TRUE(
    sf.bestMove(
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

// legalMoves() and applyMove() are what who_is_owner_of_move()'s
// reconciliation ladder and is_checkmate() are built on -- invariant (a)
// (never write the robot's own move ahead of camera evidence) and the
// checkmate/"killking" decision both depend on these parsing the engine's
// text output correctly. A parse miss that legalMoves() reports as success
// with an empty list looks identical to a genuine checkmate to
// DecisionPlugin::is_checkmate(), so this is worth covering directly rather
// than only indirectly through DecisionPlugin (which has no live-graph
// test at all, for the same reason ArmPlugin does not).
TEST(StockfishProcess, EnumeratesTwentyLegalMovesFromTheOpeningPosition)
{
  StockfishProcess sf;
  ASSERT_TRUE(sf.start(std::chrono::seconds(5))) << sf.lastError();

  std::vector<std::string> moves;
  ASSERT_TRUE(
    sf.legalMoves(
      "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
      std::chrono::seconds(5), moves)) << sf.lastError();
  EXPECT_EQ(moves.size(), 20u);
  sf.stop();
}

TEST(StockfishProcess, ReportsNoLegalMovesOnAMatePosition)
{
  StockfishProcess sf;
  ASSERT_TRUE(sf.start(std::chrono::seconds(5))) << sf.lastError();

  std::vector<std::string> moves;
  // Fool's mate: 1. f3 e5 2. g4 Qh4#. White to move, no legal moves --
  // exactly the position is_checkmate() must recognise as "killking", not
  // silently misreport via a parse miss (see Finding 1 in the task-5
  // review: legalMoves() used to trust "nothing parsed as a move" as
  // equivalent to "the engine reported zero legal moves", which are not
  // the same claim).
  ASSERT_TRUE(
    sf.legalMoves(
      "rnb1kbnr/pppp1ppp/8/4p3/6Pq/5P2/PPPPP2P/RNBQKBNR w KQkq - 1 3",
      std::chrono::seconds(5), moves)) << sf.lastError();
  EXPECT_TRUE(moves.empty());
  sf.stop();
}

TEST(StockfishProcess, AppliesAMoveAndReturnsTheResultingFen)
{
  StockfishProcess sf;
  ASSERT_TRUE(sf.start(std::chrono::seconds(5))) << sf.lastError();

  std::string fen_out;
  ASSERT_TRUE(
    sf.applyMove(
      "rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1",
      "e2e4", std::chrono::seconds(5), fen_out)) << sf.lastError();
  // No black pawn is adjacent on rank 4, so this build's "d" omits the en
  // passant target square entirely rather than naming an uncapturable one.
  EXPECT_EQ(fen_out, "rnbqkbnr/pppppppp/8/8/4P3/8/PPPP1PPP/RNBQKBNR b KQkq - 0 1");
  sf.stop();
}
