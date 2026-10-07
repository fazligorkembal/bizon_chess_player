#ifndef BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONDITION__POLL_THROTTLE_HPP_
#define BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONDITION__POLL_THROTTLE_HPP_

#include <chrono>
#include <optional>

namespace bizon_behavior_clients
{
/// Decides, given a monotonic timestamp, whether enough time has passed
/// since the last real poll to justify another one. Extracted as a pure,
/// ROS-free class (per this repo's convention that logic which can be
/// tested directly should be extracted so that it is -- see fen_utils.hpp)
/// so the rate-limiting decision can be unit tested with synthetic
/// timestamps: no real sleeping, no live lifecycle manager.
///
/// Used by IsSystemActiveNode to bound how often it calls the real
/// is_active service. IsSystemActiveNode sits behind a ReactiveFallback,
/// which ticks it on every single BT tick (about every 10ms) for as long as
/// the system stays paused, so without this a pause held for any length
/// of time would drive on the order of 100 service round-trips -- and the
/// log lines each one produces -- per second. Gating real polls to at most
/// once per min_interval trades a bounded amount of added detection
/// latency (at most min_interval, on both the pause and the resume side)
/// for that. That trade is deliberately one-sided: min_interval only needs
/// to stay well under the several seconds a single arm move takes, which is
/// the latency this class must not reintroduce.
class PollThrottle
{
public:
  explicit PollThrottle(std::chrono::steady_clock::duration min_interval)
  : min_interval_(min_interval)
  {}

  /// Returns true if a fresh poll is due at `now`, and if so records `now`
  /// as the time of that poll. The very first call always polls (there is
  /// nothing to throttle against yet), so the first paused tick is always
  /// seen immediately rather than delayed by a full interval.
  bool shouldPollNow(std::chrono::steady_clock::time_point now)
  {
    if (!last_poll_.has_value() || now - *last_poll_ >= min_interval_) {
      last_poll_ = now;
      return true;
    }
    return false;
  }

private:
  std::chrono::steady_clock::duration min_interval_;
  std::optional<std::chrono::steady_clock::time_point> last_poll_;
};

}  // namespace bizon_behavior_clients

#endif  // BIZON_BEHAVIOR_CLIENTS__PLUGINS__CONDITION__POLL_THROTTLE_HPP_
