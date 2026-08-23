#include "bizon_chess/fen.hpp"

#include <cctype>

namespace bizon_chess
{

std::vector<std::string> occupiedSquares(const std::string & fen)
{
  std::vector<std::string> squares;
  if (fen.empty()) {
    return squares;
  }

  const std::string board = fen.substr(0, fen.find(' '));

  int rank = 8;
  int file = 0;
  for (const char c : board) {
    if (c == '/') {
      if (file != 8) { return {}; }   // short or long rank: malformed
      rank--;
      file = 0;
      if (rank < 1) { return {}; }
    } else if (c >= '1' && c <= '8') {
      file += c - '0';
      if (file > 8) { return {}; }
    } else if (std::isalpha(static_cast<unsigned char>(c))) {
      if (file > 7) { return {}; }
      squares.push_back(std::string(1, static_cast<char>('a' + file)) + std::to_string(rank));
      file++;
    } else {
      return {};
    }
  }

  if (rank != 1 || file != 8) {
    return {};
  }
  return squares;
}

}  // namespace bizon_chess
