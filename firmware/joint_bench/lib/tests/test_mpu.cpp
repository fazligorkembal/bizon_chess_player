#include <gtest/gtest.h>

#include <map>

#include "jb/mpu.hpp"

using jb::mpu::Chip;

namespace
{
std::map<uint8_t, uint8_t> writes(const jb::mpu::Plan & p)
{
  std::map<uint8_t, uint8_t> m;
  for (uint8_t i = 0; i < p.count; ++i) {
    m[p.writes[i].reg] = p.writes[i].value;
  }
  return m;
}
}  // namespace

TEST(Mpu, IdentifiesTheSupportedChipsAndNothingElse)
{
  EXPECT_EQ(jb::mpu::identify(0x68), Chip::Mpu6050);
  EXPECT_EQ(jb::mpu::identify(0x70), Chip::Mpu6500);
  EXPECT_EQ(jb::mpu::identify(0x71), Chip::Mpu6500);
  EXPECT_EQ(jb::mpu::identify(0x98), Chip::Unknown);
  EXPECT_EQ(jb::mpu::identify(0x00), Chip::Unknown);
}

TEST(Mpu, PlansTheDefaultConfigurationForAnMpu6050)
{
  jb::mpu::Plan p{};
  ASSERT_EQ(jb::mpu::plan_config(Chip::Mpu6050, jb::mpu::kDefault, &p), nullptr);
  const auto w = writes(p);
  EXPECT_EQ(w.at(jb::mpu::reg::kPwrMgmt1), 0x01);   // gyro PLL clock
  EXPECT_EQ(w.at(jb::mpu::reg::kSmplrtDiv), 0);     // 1 kHz
  EXPECT_EQ(w.at(jb::mpu::reg::kConfig), 1);        // 184 Hz, nearest to 188
  EXPECT_EQ(w.at(jb::mpu::reg::kAccelConfig), 0x08);  // +-4 g
  EXPECT_EQ(w.at(jb::mpu::reg::kIntPinCfg), 0x10);
  EXPECT_EQ(w.at(jb::mpu::reg::kIntEnable), 0x01);
  EXPECT_EQ(w.count(jb::mpu::reg::kAccelConfig2), 0u);  // not an MPU-6050 register
  EXPECT_EQ(p.accepted.rate_hz, 1000);
  EXPECT_EQ(p.accepted.dlpf_hz, 184);
  EXPECT_EQ(p.lsb_per_g, 8192);
}

TEST(Mpu, WritesTheSeparateAccelFilterOnAnMpu6500)
{
  jb::mpu::Plan p{};
  ASSERT_EQ(jb::mpu::plan_config(Chip::Mpu6500, {500, 100, 2}, &p), nullptr);
  const auto w = writes(p);
  EXPECT_EQ(w.at(jb::mpu::reg::kSmplrtDiv), 1);     // 500 Hz
  EXPECT_EQ(w.at(jb::mpu::reg::kAccelConfig2), 2);  // 99 Hz
  EXPECT_EQ(p.accepted.dlpf_hz, 99);
  EXPECT_EQ(p.lsb_per_g, 16384);
}

TEST(Mpu, RejectsWhatTheChipCannotDo)
{
  jb::mpu::Plan p{};
  EXPECT_STREQ(jb::mpu::plan_config(Chip::Mpu6050, {300, 188, 4}, &p), "rate must divide 1000 Hz");
  EXPECT_STREQ(jb::mpu::plan_config(Chip::Mpu6050, {2000, 188, 4}, &p), "rate must divide 1000 Hz");
  EXPECT_STREQ(jb::mpu::plan_config(Chip::Mpu6050, {2, 188, 4}, &p), "rate must divide 1000 Hz");
  EXPECT_STREQ(jb::mpu::plan_config(Chip::Mpu6050, {1000, 188, 3}, &p), "fs must be 2, 4, 8 or 16");
  EXPECT_STREQ(jb::mpu::plan_config(Chip::Unknown, jb::mpu::kDefault, &p), "unknown chip");
}

TEST(Mpu, DecodesTheBigEndianBlock)
{
  const uint8_t block[14] = {0x00, 0x01, 0xFF, 0xFF, 0x80, 0x00, 0x12, 0x34,
    0x7F, 0xFF, 0x00, 0x00, 0xC0, 0x00};
  const jb::mpu::Raw r = jb::mpu::parse_block(block);
  EXPECT_EQ(r.accel[0], 1);
  EXPECT_EQ(r.accel[1], -1);
  EXPECT_EQ(r.accel[2], -32768);
  EXPECT_EQ(r.temp, 0x1234);
  EXPECT_EQ(r.gyro[0], 32767);
  EXPECT_EQ(r.gyro[1], 0);
  EXPECT_EQ(r.gyro[2], -16384);
}
