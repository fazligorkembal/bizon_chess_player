#include "jb/step_gen.hpp"

namespace jb
{

bool StepGen::start(uint32_t count, uint32_t period_us, uint64_t start_us)
{
  if (count == 0 || period_us < 2) {
    active_ = false;
    return false;
  }
  count_ = count;
  period_us_ = period_us;
  next_us_ = start_us;
  issued_ = 0;
  active_ = true;
  return true;
}

StepGen::Action StepGen::poll(uint64_t now_us)
{
  if (!active_) {
    return Action::None;
  }
  if (issued_ == count_) {
    active_ = false;
    return Action::Done;
  }
  if (now_us < next_us_) {
    return Action::None;
  }
  if (now_us - next_us_ > period_us_ / 2) {
    active_ = false;
    return Action::Late;
  }
  ++issued_;
  next_us_ += period_us_;
  return Action::Step;
}

}  // namespace jb
