#pragma once

#include <cstdint>
#include <string_view>

namespace jb
{

// Collects received bytes into command lines. Either CR or LF ends a line, and empty lines
// are skipped, so CR, LF and CRLF terminals all work. A line longer than the buffer is
// discarded whole and reported once at its terminator: executing its first 64 bytes, or
// its tail as a second command, would run something the user never typed.
class LineAssembler
{
public:
  static constexpr uint32_t kMax = 64;

  enum class Result { None, Line, Overflow };

  // On Line, line() holds the text without its terminator until the next call to feed().
  Result feed(char c);
  std::string_view line() const {return std::string_view(buf_, line_len_);}

private:
  char buf_[kMax] {};
  uint32_t len_ = 0;
  uint32_t line_len_ = 0;
  bool overflow_ = false;
};

}  // namespace jb
