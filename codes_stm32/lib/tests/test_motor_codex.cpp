#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <string_view>

#include "jb/command.hpp"
#include "jb/step_gen.hpp"
#include "jb/tmc_uart.hpp"

namespace
{
using MotorAction = jb::StepGen::Action;
}

TEST(MotorCodex, RequirementFirstSpinIssuesExactly1600EdgesAt400Hz)
{
  // Plan: 200 full steps at 1/8, 400 STEP rising edges/s.
  // Hand calculation: 200*8=1600; period=1000000/400=2500 us.
  jb::StepGen gen;
  constexpr uint64_t start = 123456;
  ASSERT_TRUE(gen.start(1600, 2500, start));
  for (uint32_t i = 0; i < 1600; ++i) {
    const uint64_t due = start + uint64_t{i} * 2500;
    EXPECT_EQ(gen.poll(due - 1), MotorAction::None) << i;
    ASSERT_EQ(gen.poll(due), MotorAction::Step) << i;
    EXPECT_EQ(gen.issued(), i + 1);
    // Repeated polls cannot issue another edge at the same timestamp.
    EXPECT_EQ(gen.poll(due), i == 1599 ? MotorAction::Done : MotorAction::None);
  }
  EXPECT_FALSE(gen.active());
  EXPECT_EQ(gen.issued(), 1600u);
  EXPECT_EQ(gen.poll(start + 4000000), MotorAction::None);
}

TEST(MotorCodex, RequirementMaximumSlowMoveDoesNotTruncateElapsedTimeTo32Bits)
{
  // 64000 edges at 1 Hz span 63999 s, exceeding uint32_t microseconds.
  // Also start just before that boundary to exercise the first crossing.
  jb::StepGen gen;
  constexpr uint64_t start = (uint64_t{1} << 32) - 10;
  ASSERT_TRUE(gen.start(64000, 1000000, start));
  for (uint32_t i = 0; i < 64000; ++i) {
    const uint64_t due = start + uint64_t{i} * 1000000;
    EXPECT_EQ(gen.poll(due - 1), MotorAction::None) << i;
    ASSERT_EQ(gen.poll(due), MotorAction::Step) << i;
  }
  EXPECT_EQ(gen.poll(start + uint64_t{63999} * 1000000 + 5), MotorAction::Done);
  EXPECT_EQ(gen.issued(), 64000u);
}

TEST(MotorCodex, RequirementLateFirstEdgeNeverStartsTheMotor)
{
  jb::StepGen gen;
  ASSERT_TRUE(gen.start(1600, 2500, 10000));
  EXPECT_EQ(gen.poll(11251), MotorAction::Late);
  EXPECT_EQ(gen.issued(), 0u);
  EXPECT_FALSE(gen.active());
  EXPECT_EQ(gen.poll(12500), MotorAction::None);
}

TEST(MotorCodex, RequirementOddPeriodUsesTheExactHalfPeriodLateBoundary)
{
  // 333 Hz -> integer period 3003 us. 1501 us is below half, 1502 is above.
  jb::StepGen accepted;
  jb::StepGen rejected;
  ASSERT_TRUE(accepted.start(2, 3003, 10000));
  ASSERT_TRUE(rejected.start(2, 3003, 10000));
  EXPECT_EQ(accepted.poll(11501), MotorAction::Step);
  EXPECT_EQ(rejected.poll(11502), MotorAction::Late);
  EXPECT_EQ(rejected.issued(), 0u);
}

TEST(MotorCodex, RequirementNewMoveAfterAbortStartsWithANewCountAndDeadline)
{
  jb::StepGen gen;
  ASSERT_TRUE(gen.start(1600, 2500, 100));
  EXPECT_EQ(gen.poll(100), MotorAction::Step);
  EXPECT_EQ(gen.poll(3851), MotorAction::Late);  // second due=2600, lateness=1251
  ASSERT_TRUE(gen.start(100, 2500, 10000));
  EXPECT_EQ(gen.issued(), 0u);
  EXPECT_EQ(gen.count(), 100u);
  EXPECT_EQ(gen.poll(9999), MotorAction::None);
  EXPECT_EQ(gen.poll(10000), MotorAction::Step);
}

TEST(MotorCodex, RequirementIholdIrunPolicyChecksEveryCurrentAndDelayCombination)
{
  // Rev.1.09 p.28: IHOLD[4:0], IRUN[12:8], IHOLDDELAY[19:16].
  // Plan: IRUN <= 16 and IHOLD <= IRUN; all 16 delay values are defined.
  for (uint32_t delay = 0; delay < 16; ++delay) {
    for (uint32_t run = 0; run < 32; ++run) {
      for (uint32_t hold = 0; hold < 32; ++hold) {
        const uint32_t value = (delay << 16) | (run << 8) | hold;
        EXPECT_EQ(jb::tmc2209::bench_write_refusal(0x10, value) == nullptr,
          run <= 16 && hold <= run) << value;
      }
    }
  }
}

TEST(MotorCodex, RequirementEveryReservedIholdIrunBitIsRejectedIndividually)
{
  // Derive the legal mask from the datasheet field locations, not implementation output.
  constexpr uint32_t defined = (0x1Fu << 0) | (0x1Fu << 8) | (0xFu << 16);
  for (unsigned bit = 0; bit < 32; ++bit) {
    const uint32_t flag = uint32_t{1} << bit;
    if ((flag & defined) == 0) {
      EXPECT_NE(jb::tmc2209::bench_write_refusal(0x10, 0x00011008u | flag), nullptr)
        << bit;
    }
  }
}

TEST(MotorCodex, RequirementReadbackPolicyRejectsEveryReservedGconfBit)
{
  // Rev.1.09 p.23: only bits 0..9 exist; readback uses a separate public entry point.
  for (unsigned bit = 10; bit < 32; ++bit) {
    EXPECT_NE(jb::tmc2209::bench_gconf_refusal(0x145u | (uint32_t{1} << bit)), nullptr)
      << bit;
  }
}

TEST(MotorCodex, RequirementMoveParserPreservesBothSmallestSignedMovesAndRateLimits)
{
  for (int steps : {-64000, -1, 1, 64000}) {
    for (unsigned rate : {1u, 2000u}) {
      const std::string line = "move " + std::to_string(steps) + " " + std::to_string(rate);
      const jb::Command cmd = jb::parse_command(line);
      ASSERT_EQ(cmd.kind, jb::CmdKind::Move) << line;
      EXPECT_EQ(cmd.move.steps, steps);
      EXPECT_EQ(cmd.move.rate_hz, rate);
    }
  }
}

TEST(MotorCodex, RequirementMoveParserRejectsOverflowHexAndEmbeddedNul)
{
  for (const char * line : {"move -4294967296 400", "move 4294967297 400",
      "move 18446744073709551617 400", "move -18446744073709551617 400",
      "move 1 4294967297", "move 0x10 400", "move -0x10 400", "move 100 0x190"})
  {
    EXPECT_EQ(jb::parse_command(line).kind, jb::CmdKind::Invalid) << line;
  }
  constexpr char nul[] = "move 100 400\0stop";
  EXPECT_EQ(jb::parse_command(std::string_view(nul, sizeof(nul) - 1)).kind,
    jb::CmdKind::Invalid);
  constexpr char stop_nul[] = "stop\0move 100 400";
  EXPECT_EQ(jb::parse_command(std::string_view(stop_nul, sizeof(stop_nul) - 1)).kind,
    jb::CmdKind::Invalid);
}

// Optional integration tests compile the real app.cpp against a host HAL double.
// Enable with -DJB_MOTOR_CODEX_APP_TEST and include paths for drivers/ and a fake main.h.
// This audit used /tmp/motor_codex_stubs/main.h; normal lib/CMakeLists.txt stays HAL-free.
#ifdef JB_MOTOR_CODEX_APP_TEST
#include <gtest/gtest.h>
#include <string>
#include <vector>
#include "../../app/app.cpp"

namespace {
struct BenchFake {
  uint64_t us = 0;
  bool en_high = true;
  bool stream = false;
  uint32_t gconf = 0x145;
  uint8_t ifcnt = 0;
  unsigned reads = 0, writes = 0, probes = 0;
  unsigned fail_read = 0;
  int fail_reg = -1;
  bool fail_write = false;
  uint8_t increment = 1;
  std::string tx;
  std::vector<std::string> events;
} f;
}
void HAL_GPIO_WritePin(GPIO_TypeDef * port, uint16_t pin, GPIO_PinState value) {
  if (port == TMC_EN_GPIO_Port && pin == TMC_EN_Pin) {
    f.en_high = value == GPIO_PIN_SET;
    f.events.push_back(f.en_high ? "EN high" : "EN low");
  }
}
void HAL_GPIO_TogglePin(GPIO_TypeDef *, uint16_t) {}
// Added by Claude on 2026-11-10 when endstop 1 (B10) polling was added to app.cpp.
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *, uint16_t) {return GPIO_PIN_SET;}
uint32_t HAL_GetTick() {return static_cast<uint32_t>(f.us / 1000);}
namespace jb {
void clock_init() {}
uint64_t now_us() {return f.us++;}
const char * imu_probe(const mpu::Config &) {++f.probes; return nullptr;}
const char * imu_configure(const mpu::Config &) {++f.probes; return nullptr;}
bool imu_present() {return true;}
mpu::Chip imu_chip() {return mpu::Chip::Unknown;}
uint8_t imu_who_am_i() {return 0;}
uint8_t imu_address() {return 0x68;}
const mpu::Plan & imu_plan() {static mpu::Plan p{}; return p;}
void imu_stream(bool on) {f.stream = on;}
bool imu_streaming() {return f.stream;}
bool imu_pop(ImuSample *) {return false;}
ImuCounters imu_counters() {return {};}
int imu_scan(uint8_t *, int) {++f.probes; return 0;}
bool usb_send(const char * s, uint32_t n) {f.tx.append(s, n); return true;}
void usb_pump() {}
bool usb_rx_pop(char *) {return false;}
uint32_t usb_tx_dropped() {return 0;}
uint32_t usb_rx_dropped() {return 0;}
void usb_rx_isr(const uint8_t *, uint32_t) {}
void usb_tx_done_isr() {}
TmcStatus tmc_read(uint8_t reg, uint32_t *value, tmc2209::ReplyStatus *) {
  ++f.reads;
  f.events.push_back(reg == 0 ? "read GCONF" : reg == 2 ? "read IFCNT" : "read other");
  if (f.fail_read == f.reads || f.fail_reg == reg) return TmcStatus::NoReply;
  *value = reg == 0 ? f.gconf : reg == 2 ? f.ifcnt : 0;
  return TmcStatus::Ok;
}
TmcStatus tmc_write(uint8_t reg, uint32_t value) {
  ++f.writes;
  f.events.push_back(reg == 0x10 ? "write IHOLD_IRUN" : "write other");
  if (f.fail_write) return TmcStatus::SendFailed;
  f.ifcnt = static_cast<uint8_t>(f.ifcnt + f.increment);
  if (reg == 0) f.gconf = value;
  return TmcStatus::Ok;
}
}
class MotorAppCodex : public ::testing::Test {
protected:
  void SetUp() override {f = BenchFake{}; g_motor = Motor{}; g_rx = jb::LineAssembler{};}
  void ready() {cmd_tmc_write(0x10, 0x11008); ASSERT_TRUE(g_motor.have_current);}
  void enabled() {ready(); cmd_tmc_enable(true); ASSERT_FALSE(f.en_high);}
};
TEST_F(MotorAppCodex, RequirementNoEnableBeforeApprovedCountedCurrent) {
  cmd_tmc_enable(true);
  EXPECT_TRUE(f.en_high); EXPECT_EQ(f.writes, 0u); EXPECT_FALSE(g_motor.enabled);
}
TEST_F(MotorAppCodex, RequirementEachEnableCountsAFreshCurrentWriteBeforeLoweringEn) {
  ready(); f.events.clear(); f.ifcnt = 255;
  cmd_tmc_enable(true);
  // Updated by Claude on 2026-11-10: IOIN (SPREAD) is read and GSTAT cleared and checked before
  // the current write, as Codex agreed in step3-motor-firmware turn 2 and drv-status-bits.
  const std::vector<std::string> expected{"read GCONF", "read other", "read IFCNT", "write other",
    "read IFCNT", "read other", "read IFCNT", "write IHOLD_IRUN", "read IFCNT", "EN low"};
  // Two counted writes now (GSTAT clear, then IHOLD_IRUN): 255 -> 0 -> 1, so the wrap is still
  // exercised, by the first one.
  EXPECT_EQ(f.events, expected); EXPECT_FALSE(f.en_high); EXPECT_EQ(f.ifcnt, 1);
  cmd_tmc_enable(false); f.events.clear();
  cmd_tmc_enable(true); EXPECT_EQ(f.events, expected);
}
TEST_F(MotorAppCodex, RequirementEveryEnableReadFailureLeavesEnHigh) {
  for (unsigned failure : {1u, 2u, 3u}) {
    SCOPED_TRACE(failure);
    f = BenchFake{}; g_motor = Motor{}; ready();
    f.fail_read = f.reads + failure;
    cmd_tmc_enable(true); EXPECT_TRUE(f.en_high); EXPECT_FALSE(g_motor.enabled);
  }
}
TEST_F(MotorAppCodex, RequirementEnableSendFailureLeavesEnHigh) {
  ready(); f.fail_write = true; cmd_tmc_enable(true);
  EXPECT_TRUE(f.en_high); EXPECT_FALSE(g_motor.enabled);
}
TEST_F(MotorAppCodex, RequirementUncountedOrMultiplyCountedEnableWriteLeavesEnHigh) {
  for (uint8_t increment : {0, 2, 255}) {
    SCOPED_TRACE(increment);
    f = BenchFake{}; g_motor = Motor{}; ready(); f.increment = increment;
    cmd_tmc_enable(true); EXPECT_TRUE(f.en_high); EXPECT_FALSE(g_motor.enabled);
  }
}
TEST_F(MotorAppCodex, RequirementDriverResetWhileOffRequiresReconfiguration) {
  ready(); f.gconf = 0x101; f.ifcnt = 0;
  cmd_tmc_enable(true); EXPECT_TRUE(f.en_high); EXPECT_FALSE(g_motor.enabled);
}
TEST_F(MotorAppCodex, RequirementConfigWritesAreRefusedWhileEnabled) {
  enabled(); const unsigned writes = f.writes;
  cmd_tmc_write(0, 0x145); cmd_tmc_write(0x10, 0x10008);
  EXPECT_EQ(f.writes, writes); EXPECT_FALSE(f.en_high);
}
TEST_F(MotorAppCodex, RequirementBlockingCommandsAreRefusedDuringMotion) {
  enabled(); cmd_move({1600, 400});
  const unsigned reads = f.reads, writes = f.writes, probes = f.probes;
  for (const char * line : {"i2cscan", "imu info", "imu cfg rate=1000", "stream on",
       "tmc read 0", "tmc write 0 0x145", "tmc en on", "move -100 400"}) {
    SCOPED_TRACE(line); f.tx.clear(); handle(jb::parse_command(line));
    EXPECT_NE(f.tx.find("busy: moving"), std::string::npos);
    EXPECT_TRUE(g_motor.gen.active());
  }
  EXPECT_EQ(f.reads, reads); EXPECT_EQ(f.writes, writes); EXPECT_EQ(f.probes, probes);
}
TEST_F(MotorAppCodex, RequirementNonblockingCommandsStayAvailableDuringMotion) {
  enabled(); cmd_move({1600, 400}); const unsigned reads = f.reads;
  for (const char * line : {"help", "ver", "stats", "mark test", "stream off", "unknown"}) {
    SCOPED_TRACE(line); f.tx.clear(); handle(jb::parse_command(line));
    EXPECT_EQ(f.tx.find("busy: moving"), std::string::npos);
    EXPECT_TRUE(g_motor.gen.active());
  }
  EXPECT_EQ(f.reads, reads);
}
TEST_F(MotorAppCodex, RequirementStopAndDisableWorkEvenWithStreamingAndBrokenUart) {
  enabled(); cmd_move({1600, 400}); f.stream = true; f.fail_reg = 0;
  const unsigned reads = f.reads;
  handle(jb::parse_command("stop")); EXPECT_FALSE(g_motor.gen.active());
  ASSERT_TRUE(g_motor.gen.start(1600, 2500, f.us));
  handle(jb::parse_command("tmc en off"));
  EXPECT_TRUE(f.en_high); EXPECT_FALSE(g_motor.gen.active()); EXPECT_EQ(f.reads, reads);
}
// Red regression tests: accepted plan B1/B4/B5 and "boot and any error leave EN high".
TEST_F(MotorAppCodex, RequirementUartFailureWhileEnabledDisablesBridges) {
  enabled(); f.fail_reg = 0x6a;
  handle(jb::parse_command("tmc read 0x6a"));
  EXPECT_TRUE(f.en_high); EXPECT_FALSE(g_motor.enabled);
}
TEST_F(MotorAppCodex, RequirementLateMotionErrorDisablesBridges) {
  enabled(); cmd_move({1600, 400}); f.us += 1251;
  motor_poll(); EXPECT_FALSE(g_motor.gen.active());
  EXPECT_TRUE(f.en_high); EXPECT_FALSE(g_motor.enabled);
}
TEST_F(MotorAppCodex, RequirementLowCurrentFirstSpinDoesNotEnableStealthChop) {
  ready(); f.gconf = 0x141;
  cmd_tmc_enable(true); EXPECT_TRUE(f.en_high); EXPECT_FALSE(g_motor.enabled);
}
TEST_F(MotorAppCodex, RequirementDriverResetDuringMotionIsDetectedAndDisablesBridges) {
  enabled(); cmd_move({1600, 400});
  f.gconf = 0x101; f.ifcnt = 0;
  const uint64_t start = f.us;
  for (unsigned i = 0; i <= 1600; ++i) {f.us = start + uint64_t{i} * 2500; app_loop();}
  // Serve an entire four-second move: the plan gives no particular polling deadline.
  // A driver reset must at least be noticed within the move, rather than silently completing it.
  EXPECT_TRUE(f.en_high); EXPECT_FALSE(g_motor.enabled); EXPECT_FALSE(g_motor.gen.active());
}

#endif  // JB_MOTOR_CODEX_APP_TEST
