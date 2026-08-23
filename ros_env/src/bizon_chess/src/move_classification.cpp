#include "bizon_chess/move_classification.hpp"

#include <cctype>
#include <sstream>

namespace bizon_chess
{

std::string classifyMoveType(const std::string & fen, const std::string & move)
{
  const std::string board_part = fen.substr(0, fen.find(' '));

  std::stringstream ss(fen);
  std::string token;
  std::string active_color, castle_rights, en_passant;
  ss >> token;
  ss >> active_color;
  ss >> castle_rights;
  ss >> en_passant;

  const std::string from = move.substr(0, 2);
  const std::string to = move.substr(2, 2);

  // Find the piece on the source square.
  const int from_file = from[0] - 'a';
  const int from_rank = 8 - (from[1] - '0');
  const int from_index = from_rank * 8 + from_file;
  int idx = 0;
  char source_piece = '.';
  for (char c : board_part) {
    if (c == '/') {
      continue;
    }
    if (isdigit(c)) {
      if (idx + (c - '0') > from_index) {
        break;
      }
      idx += (c - '0');
    } else {
      if (idx == from_index) {
        source_piece = c;
        break;
      }
      idx++;
    }
  }

  if (source_piece == 'K' && from == "e1") {
    if (to == "g1" && castle_rights.find('K') != std::string::npos) {
      return "short_castle";
    }
    if (to == "c1" && castle_rights.find('Q') != std::string::npos) {
      return "long_castle";
    }
  }

  if (source_piece == 'k' && from == "e8") {
    if (to == "g8" && castle_rights.find('k') != std::string::npos) {
      return "short_castle";
    }
    if (to == "c8" && castle_rights.find('q') != std::string::npos) {
      return "long_castle";
    }
  }

  if (move.size() == 5) {
    const int file = move[2] - 'a';
    const int rank = 8 - (move[3] - '0');
    const int target_index = rank * 8 + file;
    int current_index = 0;
    bool target_occupied = false;

    for (char c : board_part) {
      if (c == '/') {
        continue;
      }
      if (isdigit(c)) {
        if (current_index + (c - '0') > target_index) {
          break;
        }
        current_index += (c - '0');
      } else {
        if (current_index == target_index) {
          target_occupied = true;
          break;
        }
        current_index++;
      }
    }

    if (target_occupied) {
      return "promotion_capture";
    }
    return "promotion";
  }

  const int file = move[2] - 'a';
  const int rank = 8 - (move[3] - '0');
  const int target_index = rank * 8 + file;

  int current_index = 0;
  for (char c : board_part) {
    if (c == '/') {
      continue;
    }

    if (isdigit(c)) {
      const int empty = c - '0';
      if (current_index + empty > target_index) {
        break;
      }
      current_index += empty;
    } else {
      if (current_index == target_index) {
        return "capture";
      }
      current_index++;
    }
  }

  const std::string target_square = move.substr(2, 2);
  if (target_square == en_passant) {
    return "en_passant";
  }

  return "straight";
}

}  // namespace bizon_chess
