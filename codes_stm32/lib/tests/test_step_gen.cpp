#include <gtest/gtest.h>

#include "jb/step_gen.hpp"

using Action = jb::StepGen::Action;

TEST(StepGen, IssuesEveryEdgeOnScheduleThenReportsDone)
{
  jb::StepGen gen;
  ASSERT_TRUE(gen.start(3, 2500, 1000));
  EXPECT_EQ(gen.poll(999), Action::None);
  EXPECT_EQ(gen.poll(1000), Action::Step);
  EXPECT_EQ(gen.poll(1001), Action::None);
  EXPECT_EQ(gen.poll(3500), Action::Step);
  EXPECT_EQ(gen.poll(6000), Action::Step);
  EXPECT_EQ(gen.issued(), 3u);
  EXPECT_EQ(gen.poll(6001), Action::Done);
  EXPECT_FALSE(gen.active());
  EXPECT_EQ(gen.poll(9000), Action::None);
}

TEST(StepGen, KeepsTheScheduleFixedWhenAnEdgeIsSlightlyLate)
{
  jb::StepGen gen;
  ASSERT_TRUE(gen.start(2, 1000, 0));
  EXPECT_EQ(gen.poll(400), Action::Step);  // first edge 400 us late, within half a period
  EXPECT_EQ(gen.poll(999), Action::None);  // the second is still due at 1000, not 1400
  EXPECT_EQ(gen.poll(1000), Action::Step);
}

TEST(StepGen, StopsWithLateInsteadOfBunchingEdges)
{
  jb::StepGen edge;
  ASSERT_TRUE(edge.start(10, 1000, 0));
  EXPECT_EQ(edge.poll(0), Action::Step);
  EXPECT_EQ(edge.poll(1500), Action::Step);  // exactly half a period late is still accepted
  jb::StepGen late;
  ASSERT_TRUE(late.start(10, 1000, 0));
  EXPECT_EQ(late.poll(0), Action::Step);
  EXPECT_EQ(late.poll(1501), Action::Late);
  EXPECT_FALSE(late.active());
  EXPECT_EQ(late.issued(), 1u);
  EXPECT_EQ(late.poll(1502), Action::None);
}

TEST(StepGen, StopEndsTheMoveAtOnce)
{
  jb::StepGen gen;
  ASSERT_TRUE(gen.start(5, 100, 0));
  EXPECT_EQ(gen.poll(0), Action::Step);
  gen.stop();
  EXPECT_FALSE(gen.active());
  EXPECT_EQ(gen.poll(100), Action::None);
  EXPECT_EQ(gen.issued(), 1u);
}

TEST(StepGen, RejectsAnEmptyMoveOrADegeneratePeriod)
{
  jb::StepGen gen;
  EXPECT_FALSE(gen.start(0, 1000, 0));
  EXPECT_FALSE(gen.start(5, 1, 0));
  EXPECT_FALSE(gen.active());
  EXPECT_EQ(gen.poll(0), Action::None);
}

TEST(StepGen, ReportsWhenTheNextEdgeIsDue)
{
  jb::StepGen gen;
  ASSERT_TRUE(gen.start(3, 2500, 1000));
  EXPECT_EQ(gen.next_due_us(), 1000u);
  EXPECT_EQ(gen.poll(1000), Action::Step);
  EXPECT_EQ(gen.next_due_us(), 3500u);
}
