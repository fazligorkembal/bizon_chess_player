#ifndef BIZON_BEHAVIOR_SERVERS__DEBUG_SESSION_FORMAT_HPP_
#define BIZON_BEHAVIOR_SERVERS__DEBUG_SESSION_FORMAT_HPP_

#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

// Pure, ROS-free formatting helpers for the debug logging system (see
// debug_session.hpp for the stateful side). Kept header-only and free of
// rclcpp so they can be exercised directly by test/test_debug_session_format.cpp
// without a ROS graph -- same rationale as bizon_chess/fen_utils.hpp.
namespace bizon_behaviors::debug_format
{

/// "YYYY-MM-DD HH:MM:SS.mmm" in local time, millisecond precision. Shared by
/// events.log and rosout.log so both timelines sort and read the same way.
inline std::string formatTimestamp(std::chrono::system_clock::time_point tp)
{
  using namespace std::chrono;
  const auto ms = duration_cast<milliseconds>(tp.time_since_epoch()) % 1000;
  const std::time_t t = system_clock::to_time_t(tp);
  std::tm tm_buf{};
  localtime_r(&t, &tm_buf);

  std::ostringstream out;
  out << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S");
  out << '.' << std::setfill('0') << std::setw(3) << ms.count();
  return out.str();
}

/// One events.log / rosout.log line: "<timestamp> | <category> | <body>".
/// `category` is a short fixed token ("board", "decision", "arm", "abort",
/// "error"); `body` is caller-formatted free text (key=value pairs by
/// convention, but never enforced here -- see the "no logging framework"
/// constraint in the spec).
inline std::string formatLogLine(
  std::chrono::system_clock::time_point tp, const std::string & category,
  const std::string & body)
{
  std::ostringstream out;
  out << formatTimestamp(tp) << " | " << category << " | " << body;
  return out.str();
}

/// "HH-MM-SS" for embedding in a bundle directory name (filesystem-safe --
/// no colons).
inline std::string formatTimeOfDayForPath(std::chrono::system_clock::time_point tp)
{
  const std::time_t t = std::chrono::system_clock::to_time_t(tp);
  std::tm tm_buf{};
  localtime_r(&t, &tm_buf);
  std::ostringstream out;
  out << std::put_time(&tm_buf, "%H-%M-%S");
  return out.str();
}

/// "<NNN>_<label>_<HH-MM-SS>", e.g. "003_fen_mismatch_15-43-02". `counter` is
/// zero-padded to 3 digits so bundles sort lexicographically in directory
/// listings; a game is not expected to produce more than 999 failures, and
/// if it somehow does the name just grows instead of breaking.
inline std::string formatBundleDirName(
  int counter, const std::string & label, std::chrono::system_clock::time_point tp)
{
  std::ostringstream out;
  out << std::setfill('0') << std::setw(3) << counter << '_' << label << '_'
      << formatTimeOfDayForPath(tp);
  return out.str();
}

/// The board square for cell index `board_index` (0..63), in the row-major
/// order BoardPlugin's FEN loop walks: index 0 is a8, index 63 is h1 --
/// standard FEN rank order (rank 8 down to rank 1), files a..h left to
/// right within a rank. `black_side` mirrors BoardPlugin::cameraCallback's
/// std::reverse() of class_names for the black-side robot: cell_images_ (and
/// therefore the cell crops a bundle saves) keep the camera's own left-to-
/// right/top-to-bottom order regardless of side, but which *board* square
/// that physical cell holds is reversed when the classifier's output is
/// reversed to build the FEN. So the label attached to cell_images_[i] must
/// look up index (63 - i) on the black-side robot to name the same square
/// the FEN string actually used for that crop.
inline std::string squareForCellIndex(int cell_index, bool black_side)
{
  const int board_index = black_side ? (63 - cell_index) : cell_index;
  const int rank_from_top = board_index / 8;  // 0 == rank 8
  const int file = board_index % 8;           // 0 == file a
  const int rank = 8 - rank_from_top;
  std::string square;
  square += static_cast<char>('a' + file);
  square += std::to_string(rank);
  return square;
}

/// Splits a FEN into its board field only (no side-to-move etc.); tolerates
/// a FEN that already has no space-separated fields (as the camera's FEN
/// does -- see fen_utils.hpp's side_to_move_field()).
inline std::string boardFieldOf(const std::string & fen)
{
  return fen.substr(0, fen.find(' '));
}

/// One line per square where the two board-only FENs disagree, formatted
/// "<square>: camera=<X> text=<Y>" with "empty" spelled out instead of a
/// blank value so a human doesn't misread it as a parse gap. Used to build
/// context.txt's square-level diff -- see the "Add a square-level diff"
/// refinement: on the FEN that prompted this feature (four queens on an
/// otherwise-empty back rank) this list *is* the diagnosis; reading it off
/// two raw FEN strings by eye is exactly the work this function does
/// instead.
inline std::vector<std::string> fenSquareDiff(
  const std::string & camera_fen, const std::string & text_fen)
{
  std::vector<std::string> diffs;

  auto expand = [](const std::string & board_field) {
    std::vector<char> squares;  // index 0 == a8 .. index 63 == h1, '.' == empty
    for (char c : board_field) {
      if (c == '/') {
        continue;
      }
      if (c >= '1' && c <= '8') {
        squares.insert(squares.end(), static_cast<size_t>(c - '0'), '.');
      } else {
        squares.push_back(c);
      }
    }
    squares.resize(64, '.');  // a malformed/short FEN still yields a comparable array
    return squares;
  };

  const std::vector<char> camera_squares = expand(boardFieldOf(camera_fen));
  const std::vector<char> text_squares = expand(boardFieldOf(text_fen));

  for (int i = 0; i < 64; ++i) {
    if (camera_squares[static_cast<size_t>(i)] == text_squares[static_cast<size_t>(i)]) {
      continue;
    }
    const std::string square = squareForCellIndex(i, false);
    auto describe = [](char c) { return c == '.' ? std::string("empty") : std::string(1, c); };
    std::ostringstream line;
    line << square << ": camera=" << describe(camera_squares[static_cast<size_t>(i)])
         << " text=" << describe(text_squares[static_cast<size_t>(i)]);
    diffs.push_back(line.str());
  }
  return diffs;
}

}  // namespace bizon_behaviors::debug_format

#endif  // BIZON_BEHAVIOR_SERVERS__DEBUG_SESSION_FORMAT_HPP_
