#include <gtest/gtest.h>

#include "jb/cycle_clock.hpp"

TEST(CycleClock, PassesThroughBeforeTheFirstWrap)
{
  jb::CycleClock clock(96000000);
  EXPECT_EQ(clock.extend(1000), 1000u);
  EXPECT_EQ(clock.extend(5000), 5000u);
}

TEST(CycleClock, CarriesIntoTheHighWordWhenTheCounterWraps)
{
  jb::CycleClock clock(96000000);
  clock.extend(0xFFFFFF00u);
  EXPECT_EQ(clock.extend(0x10u), 0x100000010ull);
  clock.extend(0xFFFFFFF0u);
  EXPECT_EQ(clock.extend(0x0u), 0x200000000ull);
}

TEST(CycleClock, StaysMonotonicAcrossManyWraps)
{
  jb::CycleClock clock(96000000);
  uint64_t last = 0;
  uint32_t raw = 0;
  for (int i = 0; i < 10000; ++i) {
    raw += 1500000000u;  // under one wrap per call, as the contract requires
    const uint64_t now = clock.extend(raw);
    EXPECT_GT(now, last);
    last = now;
  }
}

TEST(CycleClock, ConvertsCyclesToMicrosecondsAt96MHz)
{
  jb::CycleClock clock(96000000);
  EXPECT_EQ(clock.to_us(96), 1u);
  EXPECT_EQ(clock.to_us(95), 0u);
  EXPECT_EQ(clock.to_us(96000000ull * 3600), 3600000000ull);
}
