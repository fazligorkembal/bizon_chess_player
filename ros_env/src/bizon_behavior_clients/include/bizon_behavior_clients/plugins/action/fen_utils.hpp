#ifndef BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__FEN_UTILS_HPP_
#define BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__FEN_UTILS_HPP_

#include <string>

namespace bizon_behavior_clients
{

/// The side-to-move field of a FEN ("w" or "b"), or "" if the FEN has no such
/// field (a board-only FEN, as produced by the camera, has none).
inline std::string side_to_move_field(const std::string & fen)
{
  const size_t space_pos = fen.find(' ');
  if (space_pos != std::string::npos && space_pos + 1 < fen.size()) {
    return fen.substr(space_pos + 1, 1);
  }
  return "";
}

/// Who moves at `fen` itself: "white", "black", or "" for a FEN with no
/// side-to-move field. Use this when `fen` is the position actually on the
/// board right now.
inline std::string side_to_move(const std::string & fen)
{
  const std::string field = side_to_move_field(fen);
  if (field == "w") {
    return "white";
  }
  if (field == "b") {
    return "black";
  }
  return "";
}

/// Who moves one ply after `fen`. Use this when `fen` is a parent position and
/// the board now shows one of its children -- reading the child's own field is
/// preferable when you have the child, but the two must agree.
inline std::string side_to_move_after_one_ply(const std::string & fen)
{
  const std::string field = side_to_move_field(fen);
  if (field == "w") {
    return "black";
  }
  if (field == "b") {
    return "white";
  }
  return "";
}

}  // namespace bizon_behavior_clients

#endif  // BIZON_BEHAVIOR_CLIENTS__PLUGINS__ACTION__FEN_UTILS_HPP_
