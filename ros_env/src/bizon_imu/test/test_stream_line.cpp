#include <gtest/gtest.h>

#include "bizon_imu/stream_line.hpp"

using bizon_imu::parse_lsb_per_g;
using bizon_imu::parse_sample;

TEST(ParseSample, ReadsARecordedLine)
{
  const auto s = parse_sample("T,3001,17381236,571,-231,8341,-184,108,20");
  ASSERT_TRUE(s.has_value());
  EXPECT_EQ(s->seq, 3001u);
  EXPECT_EQ(s->t_us, 17381236u);
  EXPECT_EQ(s->accel, (std::array<int, 3>{571, -231, 8341}));
  EXPECT_EQ(s->gyro, (std::array<int, 3>{-184, 108, 20}));
}

TEST(ParseSample, AcceptsATimestampAboveInt32)
{
  const auto s = parse_sample("T,0,4294967295,0,0,0,0,0,0");
  ASSERT_TRUE(s.has_value());
  EXPECT_EQ(s->t_us, 4294967295u);
}

TEST(ParseSample, RejectsAnythingThatIsNotACompleteSample)
{
  EXPECT_FALSE(parse_sample(""));
  EXPECT_FALSE(parse_sample("OK stream on"));
  EXPECT_FALSE(parse_sample("E,123,mark"));
  EXPECT_FALSE(parse_sample("T,1,2,3,4,5,6,7"));         // one field short
  EXPECT_FALSE(parse_sample("T,1,2,3,4,5,6,7,8,9"));     // one field too many
  EXPECT_FALSE(parse_sample("T,1,2,3,4,5,6,7,8x"));      // trailing text
  EXPECT_FALSE(parse_sample("T,1,2,3,,5,6,7,8"));        // empty field
  EXPECT_FALSE(parse_sample("T,-1,2,3,4,5,6,7,8"));      // negative sequence
}

TEST(ParseLsbPerG, ReadsTheImuInfoReply)
{
  EXPECT_EQ(
    parse_lsb_per_g("OK imu chip=MPU6050 who=0x68 addr=0x68 rate=1000 dlpf=184 fs=4 lsb_per_g=8192"),
    8192);
}

TEST(ParseLsbPerG, RejectsMissingOrUnusableValues)
{
  EXPECT_FALSE(parse_lsb_per_g("ERR imu: not found"));
  EXPECT_FALSE(parse_lsb_per_g("OK imu lsb_per_g="));
  EXPECT_FALSE(parse_lsb_per_g("OK imu lsb_per_g=0"));
}
