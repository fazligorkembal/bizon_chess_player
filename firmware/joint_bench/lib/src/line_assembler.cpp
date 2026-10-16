#include "jb/line_assembler.hpp"

namespace jb
{

LineAssembler::Result LineAssembler::feed(char c)
{
  if (c == '\r' || c == '\n') {
    if (overflow_) {
      overflow_ = false;
      len_ = 0;
      return Result::Overflow;
    }
    if (len_ == 0) {
      return Result::None;
    }
    line_len_ = len_;
    len_ = 0;
    return Result::Line;
  }
  if (overflow_) {
    return Result::None;
  }
  if (len_ == kMax) {
    overflow_ = true;
    return Result::None;
  }
  buf_[len_++] = c;
  return Result::None;
}

}  // namespace jb
