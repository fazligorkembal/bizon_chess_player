#ifndef BIZON_BEHAVIOR_SERVERS__STOCKFISH_PROCESS_HPP_
#define BIZON_BEHAVIOR_SERVERS__STOCKFISH_PROCESS_HPP_

#include <chrono>
#include <string>
#include <vector>
#include <sys/types.h>

namespace bizon_behaviors
{
/// A UCI engine subprocess with deadlines on every read.
///
/// Replaces the fork()+blocking-read that lived in the BT node constructor.
/// Every read is bounded by poll(), so a wedged engine surfaces as a failed
/// action instead of freezing the behavior tree with the arm mid-air.
class StockfishProcess
{
public:
  explicit StockfishProcess(std::string binary = "stockfish");
  ~StockfishProcess();

  StockfishProcess(const StockfishProcess &) = delete;
  StockfishProcess & operator=(const StockfishProcess &) = delete;

  /// Spawn the engine and complete the uci/isready handshake.
  bool start(std::chrono::milliseconds timeout);

  /// Terminate the engine and reap it. Safe to call when not running.
  void stop();

  bool isRunning() const {return pid_ > 0;}

  /// Search `fen` to `depth` and return the bestmove token.
  /// Returns false on timeout, engine death, or a malformed reply.
  bool bestMove(
    const std::string & fen,
    int depth,
    std::chrono::milliseconds timeout,
    std::string & best_move_out);

  /// Enumerate the legal moves from `fen` in long-algebraic form, via
  /// `go perft 1` (one node per legal move, so its move list is exact and
  /// far cheaper than a real search). Used by move-owner detection to build
  /// the set of positions the board could legitimately be in next.
  /// Returns false on timeout or engine death; an empty (but true) result
  /// means `fen` has no legal moves -- checkmate or stalemate.
  bool legalMoves(
    const std::string & fen,
    std::chrono::milliseconds timeout,
    std::vector<std::string> & moves_out);

  /// Apply one legal `move` to `base_fen` and return the resulting FEN, via
  /// `position ... moves ...` followed by `d`. This is how the ladder in
  /// move-owner detection turns a candidate move into a position it can
  /// compare against the camera.
  bool applyMove(
    const std::string & base_fen,
    const std::string & move,
    std::chrono::milliseconds timeout,
    std::string & fen_out);

  /// Configure engine resource use. Call after start(). On Jetson keep
  /// threads at 1 and hash small so the engine does not contend with TensorRT.
  bool setOption(const std::string & name, const std::string & value);

  const std::string & lastError() const {return last_error_;}

private:
  bool writeLine(const std::string & line);
  /// Read until `token` appears at the start of a line, or the deadline passes.
  /// When `captured_lines` is non-null, every complete line consumed while
  /// searching -- including the matching one -- is appended to it in order.
  /// applyMove() needs this: the line it actually wants ("Fen: ...") is not
  /// the stop token ("Checkers:") but always precedes it in `d`'s output.
  bool readUntil(
    const std::string & token,
    std::chrono::steady_clock::time_point deadline,
    std::string & line_out,
    std::vector<std::string> * captured_lines = nullptr);

  std::string binary_;
  std::string last_error_;
  std::string pending_;
  int to_engine_{-1};
  int from_engine_{-1};
  pid_t pid_{-1};
};
}  // namespace bizon_behaviors

#endif  // BIZON_BEHAVIOR_SERVERS__STOCKFISH_PROCESS_HPP_
