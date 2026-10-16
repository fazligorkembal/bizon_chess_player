#include <gtest/gtest.h>

#include "jb/spsc_queue.hpp"

TEST(SpscQueue, PopsInPushOrder)
{
  jb::SpscQueue<int, 4> q;
  EXPECT_TRUE(q.push(1));
  EXPECT_TRUE(q.push(2));
  int v = 0;
  EXPECT_TRUE(q.pop(&v));
  EXPECT_EQ(v, 1);
  EXPECT_TRUE(q.pop(&v));
  EXPECT_EQ(v, 2);
  EXPECT_FALSE(q.pop(&v));
}

TEST(SpscQueue, RefusesAPushWhenFullAndKeepsTheContents)
{
  jb::SpscQueue<int, 4> q;
  for (int i = 0; i < 4; ++i) {
    EXPECT_TRUE(q.push(i));
  }
  EXPECT_FALSE(q.push(99));
  EXPECT_EQ(q.size(), 4u);
  int v = -1;
  EXPECT_TRUE(q.pop(&v));
  EXPECT_EQ(v, 0);
}

TEST(SpscQueue, SurvivesIndexWrapAround)
{
  jb::SpscQueue<uint32_t, 8> q;
  for (uint32_t i = 0; i < 100000; ++i) {
    ASSERT_TRUE(q.push(i));
    uint32_t v = 0;
    ASSERT_TRUE(q.pop(&v));
    ASSERT_EQ(v, i);
  }
  EXPECT_EQ(q.size(), 0u);
}
