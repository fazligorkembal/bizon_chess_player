#ifndef BIZON_CHESS__MOVE_CLASSIFICATION_HPP_
#define BIZON_CHESS__MOVE_CLASSIFICATION_HPP_

#include <string>

namespace bizon_chess
{
/// Classify `move` (long algebraic, e.g. "e2e4" or "e7e8q") played from
/// `fen` into one of: "short_castle", "long_castle", "promotion",
/// "promotion_capture", "capture", "en_passant", "straight".
///
/// Ported unchanged from make_decision_client_node.cpp's get_move_type():
/// this is pure FEN/move-string logic with no engine or camera involved, so
/// it moved here to be unit-tested directly rather than only indirectly
/// through a DecisionPlugin that needs a live ROS graph and a Stockfish
/// subprocess to exercise at all.
std::string classifyMoveType(const std::string & fen, const std::string & move);
}  // namespace bizon_chess

#endif  // BIZON_CHESS__MOVE_CLASSIFICATION_HPP_
