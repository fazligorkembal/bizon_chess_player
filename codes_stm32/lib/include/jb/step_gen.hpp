#pragma once

#include <cstdint>

namespace jb
{

// Paces STEP rising edges from the main loop at a fixed rate. It only decides when an edge is
// due; the caller drives the pin. Bench-only: stage C replaces it with timer PWM.
//
// Edge k (0-based) is due at start + k * period. A late edge is not made up by bunching the
// following ones, which would jump the speed: once an edge is more than half a period late
// the move ends with Late, and the caller stops.
class StepGen
{
public:
  enum class Action : uint8_t
  {
    None,  // nothing due yet, or idle
    Step,  // raise STEP now (the caller holds it high for the minimum pulse width)
    Done,  // the last edge was issued on an earlier poll; the move is over
    Late,  // an edge was due more than half a period ago; the move is over
  };

  // Starts `count` edges, `period_us` apart, the first at `start_us`. Returns false, and stays
  // idle, for a zero count or a period under 2 us.
  bool start(uint32_t count, uint32_t period_us, uint64_t start_us);

  Action poll(uint64_t now_us);

  void stop() {active_ = false;}
  bool active() const {return active_;}
  uint32_t issued() const {return issued_;}
  uint32_t count() const {return count_;}
  // When the next edge is due; meaningful only while active().
  uint64_t next_due_us() const {return next_us_;}

private:
  uint64_t next_us_ = 0;
  uint32_t period_us_ = 0;
  uint32_t count_ = 0;
  uint32_t issued_ = 0;
  bool active_ = false;
};

}  // namespace jb
