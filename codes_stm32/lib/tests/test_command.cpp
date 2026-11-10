#include <gtest/gtest.h>

#include <string>

#include "jb/command.hpp"

using jb::CmdKind;
using jb::parse_command;

TEST(Command, ParsesTheBareCommands)
{
  EXPECT_EQ(parse_command("help").kind, CmdKind::Help);
  EXPECT_EQ(parse_command("ver").kind, CmdKind::Ver);
  EXPECT_EQ(parse_command("i2cscan").kind, CmdKind::I2cScan);
  EXPECT_EQ(parse_command("stats").kind, CmdKind::Stats);
  EXPECT_EQ(parse_command("imu info").kind, CmdKind::ImuInfo);
  EXPECT_EQ(parse_command("stream on").kind, CmdKind::StreamOn);
  EXPECT_EQ(parse_command("stream off").kind, CmdKind::StreamOff);
}

TEST(Command, ToleratesExtraSpaces)
{
  EXPECT_EQ(parse_command("  stream   on  ").kind, CmdKind::StreamOn);
}

TEST(Command, RejectsUnknownWordsAndStrayArguments)
{
  EXPECT_EQ(parse_command("reboot").kind, CmdKind::Invalid);
  EXPECT_STREQ(parse_command("reboot").error, "unknown command");
  EXPECT_EQ(parse_command("ver now").kind, CmdKind::Invalid);
  EXPECT_EQ(parse_command("stream").kind, CmdKind::Invalid);
  EXPECT_EQ(parse_command("imu").kind, CmdKind::Invalid);
  EXPECT_EQ(parse_command("").kind, CmdKind::Invalid);
  EXPECT_EQ(parse_command("a b c d e f g").kind, CmdKind::Invalid);
}

TEST(Command, ParsesImuCfgKeysAndLeavesTheRestAtZero)
{
  const jb::Command c = parse_command("imu cfg rate=500 fs=8");
  ASSERT_EQ(c.kind, CmdKind::ImuCfg);
  EXPECT_EQ(c.cfg.rate_hz, 500);
  EXPECT_EQ(c.cfg.dlpf_hz, 0);
  EXPECT_EQ(c.cfg.fs_g, 8);
}

TEST(Command, RejectsBadImuCfgValues)
{
  EXPECT_STREQ(parse_command("imu cfg").error, "cfg: nothing to set");
  EXPECT_STREQ(parse_command("imu cfg rate").error, "cfg: expected key=value");
  EXPECT_STREQ(parse_command("imu cfg rate=").error, "cfg: bad value for rate");
  EXPECT_STREQ(parse_command("imu cfg rate=-5").error, "cfg: bad value for rate");
  EXPECT_STREQ(parse_command("imu cfg rate=0").error, "cfg: bad value for rate");
  EXPECT_STREQ(parse_command("imu cfg rate=99999").error, "cfg: bad value for rate");
  EXPECT_STREQ(parse_command("imu cfg fs=256").error, "cfg: bad value for fs");
  EXPECT_STREQ(parse_command("imu cfg speed=1").error, "cfg: unknown key");
}

TEST(Command, ParsesMarkWithAndWithoutALabel)
{
  EXPECT_EQ(parse_command("mark").kind, CmdKind::Mark);
  EXPECT_STREQ(parse_command("mark").label, "");
  const jb::Command c = parse_command("mark tap_board-1");
  ASSERT_EQ(c.kind, CmdKind::Mark);
  EXPECT_STREQ(c.label, "tap_board-1");
}

TEST(Command, RejectsALabelThatWouldBreakTheCsvLine)
{
  EXPECT_STREQ(parse_command("mark a,b").error, "mark: label must be [A-Za-z0-9_-]");
  EXPECT_STREQ(parse_command("mark a b").error, "mark: one label at most");
  EXPECT_STREQ(parse_command(("mark " + std::string(24, 'x')).c_str()).error,
    "mark: label too long");
  EXPECT_EQ(parse_command(("mark " + std::string(23, 'x')).c_str()).kind, CmdKind::Mark);
}

TEST(Command, ParsesTmcReadInDecimalAndHex)
{
  const jb::Command a = parse_command("tmc read 6");
  ASSERT_EQ(a.kind, CmdKind::TmcRead);
  EXPECT_EQ(a.tmc.reg, 6);
  const jb::Command b = parse_command("tmc read 0x7F");
  ASSERT_EQ(b.kind, CmdKind::TmcRead);
  EXPECT_EQ(b.tmc.reg, 0x7F);
  const jb::Command c = parse_command("tmc read 0");
  ASSERT_EQ(c.kind, CmdKind::TmcRead);
  EXPECT_EQ(c.tmc.reg, 0);
}

TEST(Command, ParsesTmcWriteWithTheFull32BitRange)
{
  const jb::Command a = parse_command("tmc write 0x00 0xFFFFFFFF");
  ASSERT_EQ(a.kind, CmdKind::TmcWrite);
  EXPECT_EQ(a.tmc.reg, 0);
  EXPECT_EQ(a.tmc.value, 0xFFFFFFFFu);
  EXPECT_EQ(parse_command("tmc write 0 4294967295").tmc.value, 0xFFFFFFFFu);
  EXPECT_EQ(parse_command("tmc write 0 0x1c1").tmc.value, 0x1C1u);
  EXPECT_EQ(parse_command("tmc write 0 0").kind, CmdKind::TmcWrite);
}

TEST(Command, RejectsBadTmcArguments)
{
  for (const char * line : {"tmc", "tmc read", "tmc read 128", "tmc read 0x80", "tmc read -1",
      "tmc read 0x", "tmc read 6 7", "tmc write 0", "tmc write 0 4294967296",
      "tmc write 0 0x100000000", "tmc write 0 0xG", "tmc write 0x80 0", "tmc peek 6"})
  {
    EXPECT_EQ(parse_command(line).kind, CmdKind::Invalid) << line;
  }
}
