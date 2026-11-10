#include "jb/debounce.hpp"

namespace jb
{

void Debouncer::reset(bool level, uint64_t now_us)
{
  stable_ = level;
  last_raw_ = level;
  since_us_ = now_us;
  raw_edges_ = 0;
  changes_ = 0;
}

bool Debouncer::feed(bool level, uint64_t now_us)
{
  if (level != last_raw_) {
    last_raw_ = level;
    since_us_ = now_us;
    ++raw_edges_;
  }
  if (level != stable_ && now_us - since_us_ >= hold_us_) {
    stable_ = level;
    ++changes_;
    return true;
  }
  return false;
}

}  // namespace jb
