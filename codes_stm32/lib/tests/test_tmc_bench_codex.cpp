#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <string_view>

#include "jb/command.hpp"
#include "jb/tmc_uart.hpp"

namespace
{

using jb::CmdKind;
using jb::parse_command;

}  // namespace

TEST(TmcBenchCodex, ParserPreservesEverySevenBitRegisterForBothCommandsAndBases)
{
  // command.hpp: registers are 0..127, decimal or prefixed hex; parsing is not write policy.
  constexpr char kHex[] = "0123456789abcdef";
  for (unsigned reg = 0; reg <= 127; ++reg) {
    std::string hex = "0x";
    hex += kHex[reg / 16];
    hex += kHex[reg % 16];
    for (const std::string & token : {std::to_string(reg), hex}) {
      const std::string read_line = "tmc read " + token;
      const jb::Command read = parse_command(read_line);
      ASSERT_EQ(read.kind, CmdKind::TmcRead) << read_line;
      EXPECT_EQ(read.tmc.reg, reg) << read_line;
      EXPECT_EQ(read.error, nullptr) << read_line;

      const std::string write_line = "tmc write " + token + " 0";
      const jb::Command write = parse_command(write_line);
      ASSERT_EQ(write.kind, CmdKind::TmcWrite) << write_line;
      EXPECT_EQ(write.tmc.reg, reg) << write_line;
      EXPECT_EQ(write.tmc.value, 0u) << write_line;
      EXPECT_EQ(write.error, nullptr) << write_line;
    }
  }
}

TEST(TmcBenchCodex, ParserPreservesUnsignedValuesAcrossBit31AndAcceptsUppercaseHex)
{
  // Mathematical uint32_t boundaries; decimal 2^31 and 2^32-1 must remain unsigned.
  const struct
  {
    const char * token;
    uint32_t expected;
  } cases[]{
    {"0000000000", 0u},
    {"010", 10u},
    {"0000000010", 10u},
    {"0X00000000", 0u},
    {"2147483647", 0x7FFFFFFFu},
    {"2147483648", 0x80000000u},
    {"4294967295", 0xFFFFFFFFu},
    {"0X80000000", 0x80000000u},
    {"0x89aBcDeF", 0x89ABCDEFu},
  };
  for (const auto & c : cases) {
    const std::string line = std::string("  tmc  write  0X7F  ") + c.token + "  ";
    const jb::Command cmd = parse_command(line);
    ASSERT_EQ(cmd.kind, CmdKind::TmcWrite) << line;
    EXPECT_EQ(cmd.tmc.reg, 127) << line;
    EXPECT_EQ(cmd.tmc.value, c.expected) << line;
    EXPECT_EQ(cmd.error, nullptr) << line;
  }
}

TEST(TmcBenchCodex, ParserRejectsValueOverflowAndMalformedTokensWithoutPrefixAcceptance)
{
  // command.hpp requires the entire value to be a decimal/hex integer in 0..2^32-1.
  for (const char * token : {"4294967296", "9999999999", "18446744073709551616",
      "0x100000000", "0x1FFFFFFFF", "-1", "+1", "0x", "0X", "0x-1", "0x+1",
      "0x1g", "1x", "1.0", "1e2", "0b10", "0xffffffffu"})
  {
    const std::string line = std::string("tmc write 0 ") + token;
    const jb::Command cmd = parse_command(line);
    EXPECT_EQ(cmd.kind, CmdKind::Invalid) << line;
    EXPECT_NE(cmd.error, nullptr) << line;
  }
}

TEST(TmcBenchCodex, ParserRejectsRegisterOverflowForReadAndWriteInsteadOfWrapping)
{
  // A register must fit seven bits before conversion into TmcArgs::reg (uint8_t).
  for (const char * token : {"128", "256", "4294967295", "4294967296", "9999999999",
      "0x80", "0x100", "0xFFFFFFFF", "0x100000000", "-1", "+0", "0X", "0x1g"})
  {
    for (const std::string & line : {std::string("tmc read ") + token,
        std::string("tmc write ") + token + " 0"})
    {
      const jb::Command cmd = parse_command(line);
      EXPECT_EQ(cmd.kind, CmdKind::Invalid) << line;
      EXPECT_NE(cmd.error, nullptr) << line;
    }
  }
}

TEST(TmcBenchCodex, ParserRejectsIncompleteAndExtraArgumentCommands)
{
  // No prefix of an incomplete or overlong command may become a read or write.
  for (const char * line : {"tmc", "tmc read", "tmc write", "tmc write 0",
      "tmc read 0 extra", "tmc write 0 1 extra", "tmc write 0 1 a b",
      "tmc write 0 1 a b c", "tmc read 0 a b c d", "tmc erase 0"})
  {
    const jb::Command cmd = parse_command(line);
    EXPECT_EQ(cmd.kind, CmdKind::Invalid) << line;
    EXPECT_NE(cmd.error, nullptr) << line;
  }
}

TEST(TmcBenchCodex, ParserUsesTheExactViewLengthAndRejectsEmbeddedNul)
{
  // LineAssembler gives parse_command a bounded view, not a NUL-terminated C string.
  const std::string backing = "tmc write 0 4294967295 junk";
  const jb::Command cmd = parse_command(
    std::string_view(backing.data(), sizeof("tmc write 0 4294967295") - 1));
  ASSERT_EQ(cmd.kind, CmdKind::TmcWrite);
  EXPECT_EQ(cmd.tmc.reg, 0);
  EXPECT_EQ(cmd.tmc.value, 0xFFFFFFFFu);

  constexpr char kEmbeddedNul[] = "tmc write 0 1\0junk";
  const jb::Command malformed = parse_command(
    std::string_view(kEmbeddedNul, sizeof(kEmbeddedNul) - 1));
  EXPECT_EQ(malformed.kind, CmdKind::Invalid);
  EXPECT_NE(malformed.error, nullptr);
}

TEST(TmcBenchCodex, ParserLeavesOtpAndTestModeRejectionToTheBenchPolicy)
{
  // Datasheet Rev. 1.09, p.24, section 5.1: OTP_PROG=0x04, OTPMAGIC=0xbd.
  // Valid syntax must still reach the policy, which must forbid this irreversible write.
  const jb::Command otp = parse_command("tmc write 0x04 0x0000bd00");
  ASSERT_EQ(otp.kind, CmdKind::TmcWrite);
  EXPECT_EQ(otp.tmc.reg, 0x04);
  EXPECT_EQ(otp.tmc.value, 0x0000BD00u);
  EXPECT_NE(jb::tmc2209::bench_write_refusal(otp.tmc.reg, otp.tmc.value), nullptr);

  // Datasheet p.23: GCONF bit 9 enables test_mode, even with ENN tied high.
  const jb::Command test_mode = parse_command("tmc write 0 0x3c1");
  ASSERT_EQ(test_mode.kind, CmdKind::TmcWrite);
  EXPECT_EQ(test_mode.tmc.value, 0x3C1u);
  EXPECT_NE(jb::tmc2209::bench_write_refusal(test_mode.tmc.reg, test_mode.tmc.value), nullptr);
}

TEST(TmcBenchCodex, PolicyRejectsTestModeForEveryCombinationOfOtherDefinedGconfBits)
{
  // Datasheet Rev. 1.09, p.23, section 5.1: GCONF has bits 0..9; test_mode is bit 9.
  // Test all 2^9 combinations of bits 0..8, with bit 9 clear and set. Updated by Claude on
  // 2026-11-10 for the motion-stage profile, as Codex's step-3 review asked: a value passes
  // only with I_scale_analog (bit 0) set and internal_Rsense (1), shaft (3) and
  // mstep_reg_select (7) clear.
  for (uint32_t lower_bits = 0; lower_bits < 512; ++lower_bits) {
    const bool profile = (lower_bits & 0x1u) != 0 && (lower_bits & 0x8Au) == 0;
    const char * refusal = jb::tmc2209::bench_write_refusal(0x00, lower_bits);
    EXPECT_EQ(refusal == nullptr, profile) << lower_bits;
    EXPECT_NE(jb::tmc2209::bench_write_refusal(0x00, lower_bits + 512), nullptr) << lower_bits;
  }
}

TEST(TmcBenchCodex, PolicyRejectsEveryOtherByteAddressRegardlessOfValue)
{
  // The policy accepts only GCONF (0x00) and IHOLD_IRUN (0x10). In particular, neither
  // OTP_PROG (0x04) nor an out-of-range address with bit 7 set may alias either of them.
  // Updated by Claude on 2026-11-10 when IHOLD_IRUN was opened for the motion stage.
  for (unsigned reg = 1; reg <= 255; ++reg) {
    if (reg == 0x10) {
      continue;
    }
    for (uint32_t value : {0u, 0x1C1u, 0xBD00u, 0xFFFFFFFFu}) {
      EXPECT_NE(jb::tmc2209::bench_write_refusal(static_cast<uint8_t>(reg), value), nullptr)
        << reg << ":" << value;
    }
  }
}
