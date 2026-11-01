#include <gtest/gtest.h>

#include <cstdint>
#include <string>

#include "jb/telemetry.hpp"
#include "jb/text_writer.hpp"

TEST(TextWriter, FormatsExtremeIntegers)
{
  char buf[64];
  jb::TextWriter w(buf, sizeof(buf));
  w.u64(0);
  w.put(' ');
  w.u64(UINT64_MAX);
  w.put(' ');
  w.i32(INT32_MIN);
  w.put(' ');
  w.hex8(0x0a);
  ASSERT_TRUE(w.ok());
  EXPECT_EQ(std::string(buf, w.size()), "0 18446744073709551615 -2147483648 0a");
}

TEST(TextWriter, NeverWritesPastCapacityAndReportsIt)
{
  char buf[8] = {'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x'};
  jb::TextWriter w(buf, 4);
  w.str("123456");
  EXPECT_FALSE(w.ok());
  EXPECT_EQ(w.size(), 4u);
  EXPECT_EQ(buf[4], 'x');
}

TEST(Telemetry, FormatsASampleLine)
{
  jb::ImuSample s{42, 123456789012ull, {-1, 2, -16384}, {0, 32767, -32768}};
  char buf[jb::kMaxLine];
  const uint32_t n = jb::format_sample(buf, sizeof(buf), s);
  EXPECT_EQ(std::string(buf, n), "T,42,123456789012,-1,2,-16384,0,32767,-32768\n");
}

TEST(Telemetry, TheLongestSampleFitsInMaxLine)
{
  jb::ImuSample s{UINT32_MAX, UINT64_MAX, {-32768, -32768, -32768}, {-32768, -32768, -32768}};
  char buf[jb::kMaxLine];
  EXPECT_EQ(jb::format_sample(buf, sizeof(buf), s), 76u);
}

TEST(Telemetry, ReturnsZeroRatherThanATruncatedLine)
{
  jb::ImuSample s{1, 2, {3, 4, 5}, {6, 7, 8}};
  char buf[10];
  EXPECT_EQ(jb::format_sample(buf, sizeof(buf), s), 0u);
}

TEST(Telemetry, FormatsAnEventLine)
{
  char buf[jb::kMaxLine];
  const uint32_t n = jb::format_event(buf, sizeof(buf), 7, "mark:tap-1");
  EXPECT_EQ(std::string(buf, n), "E,7,mark:tap-1\n");
}
