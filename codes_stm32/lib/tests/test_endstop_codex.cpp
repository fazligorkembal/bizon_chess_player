#include <gtest/gtest.h>

#include "jb/debounce.hpp"

TEST(EndstopCodex, RequirementTwoMillisecondHoldSurvivesThe32BitMicrosecondBoundary)
{
  // 2 ms hold from the endstop brief; timestamps are 64-bit microseconds.
  jb::Debouncer d(2000);
  constexpr uint64_t edge = (uint64_t{1} << 32) - 1000;
  d.reset(true, edge - 1);
  EXPECT_FALSE(d.feed(false, edge));
  EXPECT_FALSE(d.feed(false, edge + 1999));
  EXPECT_TRUE(d.feed(false, edge + 2000));
  EXPECT_FALSE(d.level());
  EXPECT_EQ(d.raw_edges(), 1u);
  EXPECT_EQ(d.changes(), 1u);
}

// Build the actual app.cpp with a host HAL header and this independent fake:
// -DJB_ENDSTOP_CODEX_APP_TEST -Icodes_stm32/drivers -I/tmp/motor_codex_stubs
// Do not also define JB_MOTOR_CODEX_APP_TEST: that enables another app.cpp harness.
// The normal HAL-free lib build runs the test above without either define.
#ifdef JB_ENDSTOP_CODEX_APP_TEST

#include <algorithm>
#include <string>
#include <vector>

#include "../../app/app.cpp"

namespace
{
struct Access
{
  bool write;
  uint8_t reg;
  uint32_t value;
  bool en_high;
};

struct EndstopFake
{
  uint64_t us = 0;
  bool es_high = true;
  bool en_high = true;
  bool streaming = false;
  uint32_t gconf = 0x145;
  uint32_t ioin = 0x21000041;
  uint32_t gstat = 0;
  uint32_t persistent_gstat = 0;
  uint8_t ifcnt = 0;
  unsigned read_count = 0;
  unsigned fail_read = 0;
  int fail_reg = -1;
  int uncounted_reg = -1;
  uint64_t read_us = 1200;  // bench's successful-read estimate, not a measured maximum
  uint64_t last_status_read = 0;
  bool missed_two_intervals = false;
  unsigned step_edges = 0;
  std::string rx;
  size_t rx_pos = 0;
  std::string tx;
  std::vector<Access> accesses;
} ef;
}

void HAL_GPIO_WritePin(GPIO_TypeDef * port, uint16_t pin, GPIO_PinState value)
{
  if (port == TMC_EN_GPIO_Port && pin == TMC_EN_Pin) {
    ef.en_high = value == GPIO_PIN_SET;
  }
  if (port == TMC_STEP_GPIO_Port && pin == TMC_STEP_Pin && value == GPIO_PIN_SET) {
    ++ef.step_edges;
  }
}
void HAL_GPIO_TogglePin(GPIO_TypeDef *, uint16_t) {}
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef * port, uint16_t pin)
{
  EXPECT_EQ(port, ES1_GPIO_Port);
  EXPECT_EQ(pin, ES1_Pin);
  return ef.es_high ? GPIO_PIN_SET : GPIO_PIN_RESET;
}
uint32_t HAL_GetTick() {return static_cast<uint32_t>(ef.us / 1000);}

namespace jb
{
void clock_init() {}
uint64_t now_us() {return ef.us++;}
const char * imu_probe(const mpu::Config &) {return nullptr;}
const char * imu_configure(const mpu::Config &)
{
  // drivers/imu.cpp apply(): HAL_Delay(100), before its subsequent register writes.
  ef.us += 100000;
  if (!ef.en_high && ef.us - ef.last_status_read >= 200000) {
    ef.missed_two_intervals = true;
  }
  return nullptr;
}
bool imu_present() {return true;}
mpu::Chip imu_chip() {return mpu::Chip::Mpu6050;}
uint8_t imu_who_am_i() {return 0x68;}
uint8_t imu_address() {return 0x68;}
const mpu::Plan & imu_plan() {static mpu::Plan p{}; return p;}
void imu_stream(bool on) {ef.streaming = on;}
bool imu_streaming() {return ef.streaming;}
bool imu_pop(ImuSample *) {return false;}
ImuCounters imu_counters() {return {};}
int imu_scan(uint8_t *, int) {return 0;}
bool usb_send(const char * s, uint32_t n) {ef.tx.append(s, n); return true;}
void usb_pump() {}
bool usb_rx_pop(char * c)
{
  if (ef.rx_pos == ef.rx.size()) {return false;}
  *c = ef.rx[ef.rx_pos++];
  return true;
}
uint32_t usb_tx_dropped() {return 0;}
uint32_t usb_rx_dropped() {return 0;}
void usb_rx_isr(const uint8_t *, uint32_t) {}
void usb_tx_done_isr() {}
TmcStatus tmc_read(uint8_t reg, uint32_t * value, tmc2209::ReplyStatus *)
{
  ++ef.read_count;
  ef.us += ef.read_us;
  ef.accesses.push_back({false, reg, 0, ef.en_high});
  if (ef.read_count == ef.fail_read || reg == ef.fail_reg) {return TmcStatus::NoReply;}
  switch (reg) {
    case 0x00: *value = ef.gconf; break;
    case 0x01: *value = ef.gstat | ef.persistent_gstat; break;
    case 0x02: *value = ef.ifcnt; break;
    case 0x06: *value = ef.ioin; break;
    default: *value = 0; break;
  }
  if (reg == 0x01 || reg == 0x00) {ef.last_status_read = ef.us;}
  return TmcStatus::Ok;
}
TmcStatus tmc_write(uint8_t reg, uint32_t value)
{
  ef.accesses.push_back({true, reg, value, ef.en_high});
  if (reg == ef.uncounted_reg) {return TmcStatus::Ok;}
  ++ef.ifcnt;
  if (reg == 0x00) {ef.gconf = value;}
  if (reg == 0x01) {ef.gstat &= ~value;}
  return TmcStatus::Ok;
}
}

class EndstopAppCodex : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ef = EndstopFake{};
    g_motor = Motor{};
    g_rx = jb::LineAssembler{};
    g_es1 = jb::Debouncer(2000);
    g_es1_watch = false;
    g_last_blink = 0;
    app_init();
  }
  void sample(bool high, uint64_t t)
  {
    ef.es_high = high;
    ef.us = t;
    endstop_poll();
  }
  void ready()
  {
    cmd_tmc_write(0x10, 0x00011008);
    ASSERT_TRUE(g_motor.have_current);
  }
  void enable()
  {
    ready();
    cmd_tmc_enable(true);
    ASSERT_TRUE(g_motor.enabled);
    ASSERT_FALSE(ef.en_high);
    ef.tx.clear();
    ef.accesses.clear();
  }
};

TEST_F(EndstopAppCodex, RequirementBootUsesEitherActualInputLevelWithoutAnEvent)
{
  for (bool high : {false, true}) {
    ef.es_high = high;
    app_init();
    EXPECT_EQ(g_es1.level(), high);
    EXPECT_EQ(g_es1.changes(), 0u);
    EXPECT_EQ(g_es1.raw_edges(), 0u);
    EXPECT_FALSE(g_es1_watch);
    EXPECT_TRUE(ef.tx.empty());
  }
}

TEST_F(EndstopAppCodex, RequirementWatchEmitsBothLevelsOnceAtTheTwoMillisecondAcceptance)
{
  handle(jb::parse_command("es watch on"));
  ASSERT_EQ(ef.tx, "OK es watch on\n");
  ef.tx.clear();
  sample(false, 10000);
  sample(false, 11999);
  EXPECT_TRUE(ef.tx.empty());
  sample(false, 12000);
  sample(false, 20000);
  sample(true, 21000);
  sample(true, 23000);
  sample(true, 30000);
  EXPECT_EQ(ef.tx, "E,12000,es1:0\nE,23000,es1:1\n");
  EXPECT_EQ(g_es1.changes(), 2u);
  EXPECT_EQ(g_es1.raw_edges(), 2u);
}

TEST_F(EndstopAppCodex, RequirementRejectedGlitchesAreCountedWithoutWatchEvents)
{
  handle(jb::parse_command("es watch on"));
  ef.tx.clear();
  sample(false, 10000);
  sample(true, 11999);
  sample(true, 20000);
  handle(jb::parse_command("es"));
  EXPECT_EQ(ef.tx, "OK es level=1 raw=1 changes=0 raw_edges=2 watch=on\n");
}

TEST_F(EndstopAppCodex, RequirementWatchOffKeepsCountsAndDoesNotReplayOldChanges)
{
  sample(false, 10000);
  sample(false, 12000);
  EXPECT_TRUE(ef.tx.empty());
  handle(jb::parse_command("es watch on"));
  ef.tx.clear();
  sample(false, 20000);
  EXPECT_TRUE(ef.tx.empty());
  handle(jb::parse_command("es watch off"));
  ASSERT_EQ(ef.tx, "OK es watch off\n");
  ef.tx.clear();
  sample(true, 21000);
  sample(true, 23000);
  handle(jb::parse_command("es"));
  EXPECT_EQ(ef.tx, "OK es level=1 raw=1 changes=2 raw_edges=2 watch=off\n");
}

TEST_F(EndstopAppCodex, RequirementReadDistinguishesRawLevelFromPendingAcceptedLevel)
{
  sample(false, 10000);
  handle(jb::parse_command("es"));
  EXPECT_EQ(ef.tx, "OK es level=1 raw=0 changes=0 raw_edges=1 watch=off\n");
}

TEST_F(EndstopAppCodex, RequirementMainLoopActuallyPollsTheEndstop)
{
  ef.es_high = false;
  ef.us = 10000;
  app_loop();
  EXPECT_TRUE(g_es1.level());
  ef.us = 12000;
  app_loop();
  EXPECT_FALSE(g_es1.level());
  EXPECT_EQ(g_es1.changes(), 1u);
}

TEST_F(EndstopAppCodex, RequirementEndstopCommandsAndEventsWorkDuringMotion)
{
  enable();
  cmd_move({1600, 400});
  ef.tx.clear();
  const unsigned reads = ef.read_count;
  handle(jb::parse_command("es watch on"));
  handle(jb::parse_command("es"));
  const uint64_t t = ef.us + 100;
  sample(false, t);
  sample(false, t + 2000);
  EXPECT_NE(ef.tx.find("OK es watch on\n"), std::string::npos);
  EXPECT_NE(ef.tx.find("OK es level=1 raw=1"), std::string::npos);
  EXPECT_NE(ef.tx.find(",es1:0\n"), std::string::npos);
  EXPECT_EQ(ef.tx.find("busy:"), std::string::npos);
  EXPECT_EQ(ef.read_count, reads);
  EXPECT_TRUE(g_motor.gen.active());
  EXPECT_FALSE(ef.en_high);
}

TEST_F(EndstopAppCodex, RequirementHelpReplyContainsTheNewCommandsAndTerminator)
{
  handle(jb::parse_command("help"));
  EXPECT_EQ(ef.tx,
    "OK help ver i2cscan stats | imu info | imu cfg rate= dlpf= fs= | "
    "stream on|off | mark [label] | tmc read <reg> | tmc write <reg> <val> | "
    "tmc en on|off | move <usteps> <hz> | stop | es [watch on|off]\n");
}

TEST_F(EndstopAppCodex, RequirementEnableRejectsLiveSpreadHigh)
{
  // TMC2209 Rev.1.09 p.23: SPREAD high inverts en_SpreadCycle; p.25: IOIN bit 8.
  ready();
  ef.ioin |= 1u << 8;
  cmd_tmc_enable(true);
  EXPECT_TRUE(ef.en_high);
  EXPECT_FALSE(g_motor.enabled);
  EXPECT_NE(ef.tx.find("SPREAD pin reads high"), std::string::npos);
}

TEST_F(EndstopAppCodex, RequirementEnableClearsLatchedGstatBeforeTheFreshCurrentWrite)
{
  ready();
  ef.gstat = 0x03;  // datasheet p.24: reset and drv_err are write-one-to-clear
  ef.accesses.clear();
  cmd_tmc_enable(true);
  ASSERT_TRUE(g_motor.enabled);
  ASSERT_EQ(ef.accesses.size(), 9u);
  EXPECT_TRUE(ef.accesses[3].write);
  EXPECT_EQ(ef.accesses[3].reg, 0x01);
  EXPECT_EQ(ef.accesses[3].value, 0x07u);
  EXPECT_FALSE(ef.accesses[5].write);
  EXPECT_EQ(ef.accesses[5].reg, 0x01);
  EXPECT_TRUE(ef.accesses[7].write);
  EXPECT_EQ(ef.accesses[7].reg, 0x10);
  EXPECT_EQ(ef.accesses[7].value, 0x00011008u);
  for (const Access & a : ef.accesses) {EXPECT_TRUE(a.en_high);}
}

TEST_F(EndstopAppCodex, RequirementEveryPersistentGstatFaultPreventsEnable)
{
  for (uint32_t fault : {1u, 2u, 4u}) {
    SCOPED_TRACE(fault);
    SetUp();
    ready();
    ef.persistent_gstat = fault;
    ef.accesses.clear();
    cmd_tmc_enable(true);
    EXPECT_TRUE(ef.en_high);
    EXPECT_FALSE(g_motor.enabled);
    EXPECT_EQ(std::count_if(ef.accesses.begin(), ef.accesses.end(),
      [](const Access & a) {return a.write && a.reg == 0x10;}), 0);
  }
}

TEST_F(EndstopAppCodex, RequirementAllSevenEnableReadFailurePathsKeepBridgesOff)
{
  // Enable: GCONF, IOIN, IFCNT, IFCNT, GSTAT, IFCNT, IFCNT.
  for (unsigned failure = 1; failure <= 7; ++failure) {
    SCOPED_TRACE(failure);
    SetUp();
    ready();
    ef.fail_read = ef.read_count + failure;
    cmd_tmc_enable(true);
    EXPECT_TRUE(ef.en_high);
    EXPECT_FALSE(g_motor.enabled);
  }
}

TEST_F(EndstopAppCodex, RequirementBothEnableWritesMustBeCounted)
{
  for (int reg : {0x01, 0x10}) {
    SCOPED_TRACE(reg);
    SetUp();
    ready();
    ef.uncounted_reg = reg;
    cmd_tmc_enable(true);
    EXPECT_TRUE(ef.en_high);
    EXPECT_FALSE(g_motor.enabled);
  }
}

TEST_F(EndstopAppCodex, RequirementEveryGstatFaultStopsBothMovingAndEnabledIdleDriver)
{
  for (bool moving : {false, true}) {
    for (uint32_t fault : {1u, 2u, 4u}) {
      SCOPED_TRACE(moving);
      SCOPED_TRACE(fault);
      SetUp();
      enable();
      // Force each of the low three GSTAT bits without changing GCONF.
      ef.gstat = fault;
      ef.us = g_motor.last_check_us + 100000;
      if (moving) {ASSERT_TRUE(g_motor.gen.start(1600, 1000000, ef.us + 500000));}
      app_loop();
      EXPECT_TRUE(ef.en_high);
      EXPECT_FALSE(g_motor.enabled);
      EXPECT_FALSE(g_motor.gen.active());
      EXPECT_NE(ef.tx.find("GSTAT fault"), std::string::npos);
    }
  }
}

TEST_F(EndstopAppCodex, RequirementSupervisionUsesSafeGapsAtThe1500MicrosecondBoundary)
{
  enable();
  ef.us = g_motor.last_check_us + 100000;
  ASSERT_TRUE(g_motor.gen.start(10, 1000000, ef.us + 1499));
  supervise();
  EXPECT_TRUE(ef.accesses.empty());
  ASSERT_TRUE(g_motor.gen.start(10, 1000000, ef.us + 1500));
  supervise();
  ASSERT_EQ(ef.accesses.size(), 1u);
  EXPECT_EQ(ef.accesses[0].reg, 0x01);
  // The 1.2 ms read leaves too little room for the second read in this gap.
  supervise();
  EXPECT_EQ(ef.accesses.size(), 1u);
  ASSERT_TRUE(g_motor.gen.start(10, 1000000, ef.us + 1500));
  supervise();
  ASSERT_EQ(ef.accesses.size(), 2u);
  EXPECT_EQ(ef.accesses[1].reg, 0x00);
}

TEST_F(EndstopAppCodex, RequirementIdleSupervisionReadsBothRegistersEvery100Milliseconds)
{
  enable();
  const uint64_t epoch = g_motor.last_check_us;
  ef.us = epoch + 99999;
  supervise();
  EXPECT_TRUE(ef.accesses.empty());
  for (uint64_t cycle = 1; cycle <= 3; ++cycle) {
    ef.us = epoch + cycle * 100000;
    supervise();
    supervise();
    ASSERT_EQ(ef.accesses.size(), cycle * 2);
    EXPECT_EQ(ef.accesses[cycle * 2 - 2].reg, 0x01);
    EXPECT_EQ(ef.accesses[cycle * 2 - 1].reg, 0x00);
  }
  EXPECT_FALSE(ef.en_high);
}

TEST_F(EndstopAppCodex, RequirementEitherSupervisionReadFailureDisablesTheDriver)
{
  for (int reg : {0x01, 0x00}) {
    SCOPED_TRACE(reg);
    SetUp();
    enable();
    ef.fail_reg = reg;
    ef.us = g_motor.last_check_us + 100000;
    supervise();
    supervise();
    EXPECT_TRUE(ef.en_high);
    EXPECT_FALSE(g_motor.enabled);
  }
}

TEST_F(EndstopAppCodex, RequirementIdleGconfChangeAlsoDisablesTheDriver)
{
  enable();
  ef.gconf = 0x101;
  ef.us = g_motor.last_check_us + 100000;
  supervise();
  supervise();
  EXPECT_TRUE(ef.en_high);
  EXPECT_FALSE(g_motor.enabled);
}

TEST_F(EndstopAppCodex, RequirementOneHertzMoveDoesNotDeferSupervisionUntilTheNextStep)
{
  enable();
  cmd_move({10, 1});
  motor_poll();
  ASSERT_EQ(ef.step_edges, 1u);
  ef.us += 100000;
  ef.gstat = 2;
  app_loop();
  EXPECT_EQ(ef.step_edges, 1u);
  EXPECT_TRUE(ef.en_high);
  EXPECT_FALSE(g_motor.gen.active());
}

TEST_F(EndstopAppCodex, Requirement500HertzMotionKeepsEveryEdgeWithSupervisionAndWatchEvents)
{
  enable();
  handle(jb::parse_command("es watch on"));
  cmd_move({1000, 500});
  const uint64_t start = ef.us;
  // 1000 pulses at 500 Hz: last rising edge at start + 999*2000 us.
  // Model the bench's 1.2 ms successful reads while polling about every 50 us.
  while (g_motor.gen.active() && ef.us - start < 2100000) {
    const uint64_t elapsed = ef.us - start;
    ef.es_high = elapsed < 500000 || elapsed >= 1000000;
    app_loop();
    ef.us += 50;
  }
  EXPECT_FALSE(g_motor.gen.active());
  EXPECT_EQ(ef.step_edges, 1000u);
  EXPECT_FALSE(ef.en_high);
  EXPECT_EQ(g_es1.changes(), 2u);
  EXPECT_NE(ef.tx.find(",es1:0\n"), std::string::npos);
  EXPECT_NE(ef.tx.find(",es1:1\n"), std::string::npos);
  EXPECT_NE(ef.tx.find("OK move done n=1000\n"), std::string::npos);
  EXPECT_EQ(ef.tx.find("ERR "), std::string::npos);
}

TEST_F(EndstopAppCodex, RequirementBlockingCommandsCannotSuspendEnabledSupervisionForTwoIntervals)
{
  enable();
  // Both complete lines fit the 256-byte RX queue and are handled in one app_loop pass.
  // Each real IMU reconfiguration waits at least 100 ms (drivers/imu.cpp:119).
  ef.rx = "imu cfg rate=1000\nimu cfg rate=1000\n";
  app_loop();
  EXPECT_FALSE(ef.missed_two_intervals)
    << "EN stayed low for >=200 ms without GSTAT/GCONF service during queued imu cfg commands";
}

#endif  // JB_ENDSTOP_CODEX_APP_TEST
