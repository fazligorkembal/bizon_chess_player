#pragma once

#include <cstdint>

namespace jb
{

// Extends the 32-bit DWT cycle counter to 64 bits and converts cycles to microseconds.
// At 84 MHz the raw counter wraps every 51.1 s, far shorter than a recording, so every
// timestamp goes through here. The extension is correct only if extend() sees the counter
// at least once per wrap; the main loop calls it every iteration, which guarantees that.
//
// Not reentrant. The firmware calls it with interrupts masked, because the IMU interrupt
// and the main loop both take timestamps and an interleaved update would lose a wrap.
class CycleClock
{
public:
  // hz must be a whole number of MHz (84 MHz here); to_us() divides by hz / 1e6.
  explicit constexpr CycleClock(uint32_t hz)
  : cycles_per_us_(hz / 1000000u) {}

  uint64_t extend(uint32_t raw)
  {
    if (raw < last_) {
      high_ += 0x100000000ull;
    }
    last_ = raw;
    return high_ + raw;
  }

  uint64_t to_us(uint64_t cycles) const {return cycles / cycles_per_us_;}

private:
  uint32_t cycles_per_us_;
  uint32_t last_ = 0;
  uint64_t high_ = 0;
};

}  // namespace jb
