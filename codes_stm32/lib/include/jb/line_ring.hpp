#pragma once

#include <cstdint>

namespace jb
{

// Byte ring holding whole text lines on their way to USB. A line that does not fit is
// dropped whole and counted: the host then sees a gap in `seq`, which it can count,
// instead of half a line spliced onto the next, which it would misparse. Used from the
// main loop only, so it needs no atomics. N must be a power of two.
template<uint32_t N>
class LineRing
{
  static_assert(N != 0 && (N & (N - 1)) == 0, "N must be a power of two");

public:
  bool push(const char * text, uint32_t len)
  {
    if (len > N - size()) {
      ++dropped_;
      return false;
    }
    for (uint32_t i = 0; i < len; ++i) {
      buf_[(head_ + i) & (N - 1)] = static_cast<uint8_t>(text[i]);
    }
    head_ += len;
    return true;
  }

  // The longest readable run that is contiguous in memory, so USB can send it in place.
  // The bytes stay valid until consume() is called, which must not happen before the
  // transfer that reads them has completed.
  uint32_t peek(const uint8_t ** data) const
  {
    const uint32_t used = size();
    if (used == 0) {
      return 0;
    }
    const uint32_t start = tail_ & (N - 1);
    const uint32_t run = (N - start < used) ? N - start : used;
    *data = &buf_[start];
    return run;
  }

  void consume(uint32_t len) {tail_ += len;}
  uint32_t size() const {return head_ - tail_;}
  uint32_t dropped() const {return dropped_;}

private:
  uint8_t buf_[N] {};
  uint32_t head_ = 0;
  uint32_t tail_ = 0;
  uint32_t dropped_ = 0;
};

}  // namespace jb
