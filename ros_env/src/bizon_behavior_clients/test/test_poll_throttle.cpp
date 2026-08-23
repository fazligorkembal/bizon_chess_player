#include <gtest/gtest.h>

#include <chrono>

#include "bizon_behavior_clients/plugins/condition/poll_throttle.hpp"

using bizon_behavior_clients::PollThrottle;

// Task 6 fix round 2 (Important 3): IsSystemActiveNode sits behind a
// ReactiveFallback, which ticks it on every BT tick (~10ms) for as long as
// the system stays paused. Without a rate limit, that alone drives ~100
// real is_active service calls -- and the log lines each one produces --
// per second for as long as an E-stop is held. PollThrottle is the pure
// rate-limiting decision extracted out of IsSystemActiveNode so it can be
// tested with synthetic timestamps: no real sleeping, no live lifecycle
// manager.

TEST(PollThrottle, FirstCallAlwaysPolls)
{
  PollThrottle gate(std::chrono::milliseconds(100));
  EXPECT_TRUE(gate.shouldPollNow(std::chrono::steady_clock::now()))
    << "the very first paused tick must be seen immediately, not delayed "
    "by a full interval";
}

TEST(PollThrottle, SuppressesCallsWithinTheInterval)
{
  PollThrottle gate(std::chrono::milliseconds(100));
  const auto t0 = std::chrono::steady_clock::now();

  ASSERT_TRUE(gate.shouldPollNow(t0));
  EXPECT_FALSE(gate.shouldPollNow(t0 + std::chrono::milliseconds(1)))
    << "a call immediately after a poll must reuse the cached decision";
  EXPECT_FALSE(gate.shouldPollNow(t0 + std::chrono::milliseconds(99)))
    << "still inside the interval, one tick before it elapses";
}

TEST(PollThrottle, PollsAgainOnceTheIntervalElapses)
{
  PollThrottle gate(std::chrono::milliseconds(100));
  const auto t0 = std::chrono::steady_clock::now();

  ASSERT_TRUE(gate.shouldPollNow(t0));
  EXPECT_TRUE(gate.shouldPollNow(t0 + std::chrono::milliseconds(100)))
    << "due exactly at the interval boundary";
  EXPECT_TRUE(gate.shouldPollNow(t0 + std::chrono::milliseconds(350)))
    << "still due even though the previous poll landed at t0+100ms, not t0: "
    "the gate must measure from the last real poll, not drift back to t0";
}
