#include <gtest/gtest.h>

#include <string>

#include "jb/line_ring.hpp"

namespace
{
template<uint32_t N>
std::string drain(jb::LineRing<N> & ring)
{
  std::string out;
  const uint8_t * p = nullptr;
  while (uint32_t n = ring.peek(&p)) {
    out.append(reinterpret_cast<const char *>(p), n);
    ring.consume(n);
  }
  return out;
}
}  // namespace

TEST(LineRing, DeliversPushedLinesInOrder)
{
  jb::LineRing<64> ring;
  EXPECT_TRUE(ring.push("a\n", 2));
  EXPECT_TRUE(ring.push("bc\n", 3));
  EXPECT_EQ(drain(ring), "a\nbc\n");
}

TEST(LineRing, DropsALineThatDoesNotFitWholeAndCountsIt)
{
  jb::LineRing<8> ring;
  EXPECT_TRUE(ring.push("12345\n", 6));
  EXPECT_FALSE(ring.push("abc\n", 4));  // 2 bytes free, 4 needed: nothing is written
  EXPECT_EQ(ring.dropped(), 1u);
  EXPECT_EQ(drain(ring), "12345\n");
}

TEST(LineRing, SplitsPeekAtTheBufferEndButKeepsTheBytes)
{
  jb::LineRing<8> ring;
  ring.push("123456", 6);
  const uint8_t * p = nullptr;
  ring.consume(ring.peek(&p));
  EXPECT_TRUE(ring.push("abcd", 4));  // wraps: 2 bytes at the end, 2 at the start
  EXPECT_EQ(ring.peek(&p), 2u);
  EXPECT_EQ(drain(ring), "abcd");
}

TEST(LineRing, PeekOnEmptyReturnsZero)
{
  jb::LineRing<8> ring;
  const uint8_t * p = nullptr;
  EXPECT_EQ(ring.peek(&p), 0u);
}
