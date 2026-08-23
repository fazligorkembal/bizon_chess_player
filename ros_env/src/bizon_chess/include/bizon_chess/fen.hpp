#ifndef BIZON_CHESS__FEN_HPP_
#define BIZON_CHESS__FEN_HPP_

#include <string>
#include <vector>

namespace bizon_chess
{
/// Algebraic names of every occupied square in a FEN's board field.
/// Accepts a full FEN or a bare board field. Returns empty on malformed input.
std::vector<std::string> occupiedSquares(const std::string & fen);
}  // namespace bizon_chess
#endif  // BIZON_CHESS__FEN_HPP_
