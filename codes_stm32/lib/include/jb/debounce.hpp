#pragma once

#include <cstdint>

namespace jb
{

// Accepts a new input level only after it has held for `hold_us`. An optical endstop does not
// bounce, but its cable runs beside the motor leads and picks up edges; the wiring guide asks
// for 1-5 ms. Raw edges are counted separately, so a test can show chatter that never got
// through.
class Debouncer
{
public:
  explicit Debouncer(uint32_t hold_us)
  : hold_us_(hold_us) {}

  // Starts from a known level, with no change pending.
  void reset(bool level, uint64_t now_us);

  // Feeds one sample. Returns true when the accepted level changes.
  bool feed(bool level, uint64_t now_us);

  bool level() const {return stable_;}
  uint32_t raw_edges() const {return raw_edges_;}
  uint32_t changes() const {return changes_;}

private:
  uint32_t hold_us_;
  bool stable_ = false;
  bool last_raw_ = false;
  uint64_t since_us_ = 0;
  uint32_t raw_edges_ = 0;
  uint32_t changes_ = 0;
};

}  // namespace jb
