#include <gtest/gtest.h>

#include "jb/debounce.hpp"

TEST(Debounce, AcceptsALevelOnlyAfterItHeld)
{
  jb::Debouncer d(2000);
  d.reset(true, 0);
  EXPECT_FALSE(d.feed(false, 100));
  EXPECT_FALSE(d.feed(false, 2099));
  EXPECT_TRUE(d.feed(false, 2100));
  EXPECT_FALSE(d.level());
  EXPECT_EQ(d.changes(), 1u);
  EXPECT_EQ(d.raw_edges(), 1u);
  EXPECT_FALSE(d.feed(false, 9000));  // no repeat while it stays
}

TEST(Debounce, IgnoresGlitchesShorterThanTheHoldButCountsThem)
{
  jb::Debouncer d(2000);
  d.reset(true, 0);
  EXPECT_FALSE(d.feed(false, 1000));
  EXPECT_FALSE(d.feed(true, 1500));   // back before 2 ms
  EXPECT_FALSE(d.feed(true, 10000));
  EXPECT_TRUE(d.level());
  EXPECT_EQ(d.changes(), 0u);
  EXPECT_EQ(d.raw_edges(), 2u);
}

TEST(Debounce, RestartsTheHoldOnEveryRawEdge)
{
  jb::Debouncer d(2000);
  d.reset(false, 0);
  EXPECT_FALSE(d.feed(true, 0));
  EXPECT_FALSE(d.feed(false, 1900));
  EXPECT_FALSE(d.feed(true, 2000));
  EXPECT_FALSE(d.feed(true, 3999));
  EXPECT_TRUE(d.feed(true, 4000));
  EXPECT_EQ(d.raw_edges(), 3u);
}

TEST(Debounce, ResetClearsCountsAndPendingChange)
{
  jb::Debouncer d(2000);
  d.reset(true, 0);
  d.feed(false, 100);
  d.reset(false, 200);
  EXPECT_FALSE(d.feed(false, 5000));
  EXPECT_EQ(d.raw_edges(), 0u);
  EXPECT_EQ(d.changes(), 0u);
}
