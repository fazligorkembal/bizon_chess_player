#include "app.h"

#include <string_view>

#include "clock.hpp"
#include "imu.hpp"
#include "jb/command.hpp"
#include "jb/debounce.hpp"
#include "jb/line_assembler.hpp"
#include "jb/step_gen.hpp"
#include "jb/telemetry.hpp"
#include "jb/text_writer.hpp"
#include "jb/tmc_uart.hpp"
#include "main.h"
#include "tmc_bus.hpp"
#include "usb_port.hpp"

#ifndef JB_VERSION
#define JB_VERSION "unknown"
#endif

namespace
{

jb::LineAssembler g_rx;
jb::mpu::Config g_cfg = jb::mpu::kDefault;
uint32_t g_last_blink = 0;

void send(const jb::TextWriter & w, const char * buf)
{
  if (w.ok()) {
    jb::usb_send(buf, w.size());
  }
}

void reply_text(bool ok, std::string_view text)
{
  char buf[256];
  jb::TextWriter w(buf, sizeof(buf));
  w.str(ok ? "OK " : "ERR ");
  w.str(text);
  w.put('\n');
  send(w, buf);
}

// "OK imu chip=MPU6050 who=0x68 addr=0x68 rate=1000 dlpf=184 fs=4 lsb_per_g=8192"
void reply_imu_info()
{
  const jb::mpu::Plan & p = jb::imu_plan();
  char buf[jb::kMaxLine];
  jb::TextWriter w(buf, sizeof(buf));
  w.str("OK imu chip=");
  w.str(jb::mpu::chip_name(jb::imu_chip()));
  w.str(" who=0x");
  w.hex8(jb::imu_who_am_i());
  w.str(" addr=0x");
  w.hex8(jb::imu_address());
  w.str(" rate=");
  w.u64(p.accepted.rate_hz);
  w.str(" dlpf=");
  w.u64(p.accepted.dlpf_hz);
  w.str(" fs=");
  w.u64(p.accepted.fs_g);
  w.str(" lsb_per_g=");
  w.u64(p.lsb_per_g);
  w.put('\n');
  send(w, buf);
}

// A missing IMU at boot is not final: the cable may simply not have been plugged in yet.
// Every command that needs the chip probes again first.
bool ensure_imu()
{
  if (jb::imu_present()) {
    return true;
  }
  if (const char * err = jb::imu_probe(g_cfg)) {
    reply_text(false, err);
    return false;
  }
  return true;
}

void cmd_i2cscan()
{
  uint8_t found[16];
  const int n = jb::imu_scan(found, 16);
  // A stuck bus was never scanned. Answering "none" would read as an empty, healthy bus and send
  // the person at the bench looking for a missing module instead of a line held low.
  if (n < 0) {
    reply_text(false, "imu: bus busy");
    return;
  }
  char buf[jb::kMaxLine];
  jb::TextWriter w(buf, sizeof(buf));
  w.str("OK i2cscan");
  if (n == 0) {
    w.str(" none");
  }
  for (int i = 0; i < n; ++i) {
    w.put(' ');
    w.hex8(found[i]);
  }
  w.put('\n');
  send(w, buf);
}

void cmd_imu_cfg(const jb::ImuCfgArgs & args)
{
  if (!ensure_imu()) {
    return;
  }
  jb::mpu::Config want = g_cfg;
  if (args.rate_hz) {want.rate_hz = args.rate_hz;}
  if (args.dlpf_hz) {want.dlpf_hz = args.dlpf_hz;}
  if (args.fs_g) {want.fs_g = args.fs_g;}
  if (const char * err = jb::imu_configure(want)) {
    reply_text(false, err);
    return;
  }
  g_cfg = want;
  reply_imu_info();
}

void cmd_mark(const char * label)
{
  // The timestamp and the edge are taken together with interrupts masked, so the E line
  // says exactly when MARK moved. Test A7 compares this against the logic analyzer.
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const uint64_t t = jb::now_us();
  HAL_GPIO_TogglePin(MARK_GPIO_Port, MARK_Pin);
  __set_PRIMASK(primask);
  char name[32];
  jb::TextWriter n(name, sizeof(name));
  n.str("mark");
  if (label[0] != '\0') {
    n.put(':');
    n.str(label);
  }
  char buf[jb::kMaxLine];
  const uint32_t len = jb::format_event(buf, sizeof(buf), t, std::string_view(name, n.size()));
  if (len) {
    jb::usb_send(buf, len);
  }
  reply_text(true, "mark");
}

// --- Motor, stage B bench ---
// EN is active low. MX_GPIO_Init drives it high before the pin becomes an output, and the fault
// handlers drive it high again, so the bridges are off whenever this code is not in control.
struct Motor
{
  bool enabled = false;
  bool have_current = false;  // a policy-approved IHOLD_IRUN was written and counted
  uint32_t ihold_irun = 0;
  uint32_t gconf = 0;  // read at enable; a driver reset changes it (pdn_disable -> 0)
  // Supervision while enabled: every kCheckIntervalUs a GSTAT read, then a GCONF read on a
  // later pass. Each read blocks about 1.2 ms, so during a move it waits for a gap of at least
  // kCheckBudgetUs before the next edge.
  uint64_t last_check_us = 0;
  uint8_t check_phase = 0;  // 0 idle, 1 GSTAT due, 2 GCONF due
  jb::StepGen gen;
};
Motor g_motor;

// Far above the datasheet's minimum STEP high and low times and DIR setup (Rev. 1.09, p.63).
constexpr uint32_t kStepHighUs = 5;
constexpr uint32_t kDirSetupUs = 5;

// While enabled, GSTAT (reset, drv_err, uv_cp) and then GCONF are re-read every
// kCheckIntervalUs. Any flag, or a GCONF that differs from the one seen at enable, switches the
// bridges off: the driver either shut down or came back from a reset at IRUN 31.
constexpr uint64_t kCheckIntervalUs = 100000;
constexpr uint64_t kCheckBudgetUs = 1500;
// At 500 Hz the gap after an edge is 2 ms, enough for one read. Faster stepping waits for timer
// PWM in stage C.
constexpr uint32_t kMaxSupervisedRateHz = 500;

void busy_wait_us(uint32_t us)
{
  const uint64_t start = jb::now_us();
  while (jb::now_us() - start < us) {
  }
}

void driver_off()
{
  HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin, GPIO_PIN_SET);
  g_motor.enabled = false;
  g_motor.gen.stop();
}

void hex32(jb::TextWriter & w, uint32_t value)
{
  w.str("0x");
  w.hex8(static_cast<uint8_t>(value >> 24));
  w.hex8(static_cast<uint8_t>(value >> 16));
  w.hex8(static_cast<uint8_t>(value >> 8));
  w.hex8(static_cast<uint8_t>(value));
}

// A driver that stops answering is a driver whose state is unknown, so a failure with the
// bridges on switches them off before reporting.
void reply_tmc_failure(jb::TmcStatus status, jb::tmc2209::ReplyStatus why)
{
  const bool was_enabled = g_motor.enabled;
  if (was_enabled) {
    driver_off();
  }
  char buf[jb::kMaxLine];
  jb::TextWriter w(buf, sizeof(buf));
  switch (status) {
    case jb::TmcStatus::Ok: return;
    case jb::TmcStatus::BadArgument: w.str("tmc: bad argument"); break;
    case jb::TmcStatus::SendFailed: w.str("tmc: send failed"); break;
    // The usual cause on the bench: VM is off, and the driver's logic runs from VM.
    case jb::TmcStatus::NoReply: w.str("tmc: no reply (VM on? wiring? pull-up?)"); break;
    case jb::TmcStatus::BadReply:
      w.str("tmc: bad reply (");
      w.str(jb::tmc2209::reply_status_name(why));
      w.put(')');
      break;
  }
  if (was_enabled) {
    w.str(", driver off");
  }
  reply_text(false, std::string_view(buf, w.size()));
}

// "OK tmc reg=0x06 val=0x21000041"
void cmd_tmc_read(uint8_t reg)
{
  uint32_t value = 0;
  jb::tmc2209::ReplyStatus why = jb::tmc2209::ReplyStatus::Ok;
  const jb::TmcStatus status = jb::tmc_read(reg, &value, &why);
  if (status != jb::TmcStatus::Ok) {
    reply_tmc_failure(status, why);
    return;
  }
  char buf[jb::kMaxLine];
  jb::TextWriter w(buf, sizeof(buf));
  w.str("OK tmc reg=0x");
  w.hex8(reg);
  w.str(" val=");
  hex32(w, value);
  w.put('\n');
  send(w, buf);
}

// A write has no reply, so it counts only if IFCNT, the driver's 8-bit count of accepted
// writes, moved by exactly one across it. Writes run one at a time, so the count is this one.
jb::TmcStatus counted_write(
  uint8_t reg, uint32_t value, uint32_t * before, uint32_t * after, bool * counted,
  jb::tmc2209::ReplyStatus * why)
{
  jb::TmcStatus status = jb::tmc_read(jb::tmc2209::kIfcnt, before, why);
  if (status == jb::TmcStatus::Ok) {
    status = jb::tmc_write(reg, value);
  }
  if (status == jb::TmcStatus::Ok) {
    status = jb::tmc_read(jb::tmc2209::kIfcnt, after, why);
  }
  *counted = status == jb::TmcStatus::Ok && static_cast<uint8_t>(*after - *before) == 1;
  return status;
}

// A write has no reply, so it counts only if IFCNT, the driver's 8-bit count of accepted
// writes, moved by exactly one across it.
// "OK tmc write reg=0x00 val=0x000001c1 ifcnt=0x05->0x06"
void cmd_tmc_write(uint8_t reg, uint32_t value)
{
  if (const char * refusal = jb::tmc2209::bench_write_refusal(reg, value)) {
    reply_text(false, refusal);
    return;
  }
  // Reconfiguring under live bridges is not something to try on a breadboard.
  if (g_motor.enabled) {
    reply_text(false, "tmc: disable the driver first (tmc en off)");
    return;
  }
  uint32_t before = 0;
  uint32_t after = 0;
  bool counted = false;
  jb::tmc2209::ReplyStatus why = jb::tmc2209::ReplyStatus::Ok;
  const jb::TmcStatus status = counted_write(reg, value, &before, &after, &counted, &why);
  if (reg == jb::tmc2209::kIholdIrun) {
    g_motor.have_current = counted;
    g_motor.ihold_irun = counted ? value : 0;
  }
  if (status != jb::TmcStatus::Ok) {
    reply_tmc_failure(status, why);
    return;
  }
  char buf[jb::kMaxLine];
  jb::TextWriter w(buf, sizeof(buf));
  w.str(counted ? "OK tmc write reg=0x" : "ERR tmc: write not counted reg=0x");
  w.hex8(reg);
  w.str(" val=");
  hex32(w, value);
  w.str(" ifcnt=0x");
  w.hex8(static_cast<uint8_t>(before));
  w.str("->0x");
  w.hex8(static_cast<uint8_t>(after));
  w.put('\n');
  send(w, buf);
}

// The driver drops back to IRUN 31 whenever VS, 5VOUT or VCC_IO dips (datasheet p.74), which
// the MCU cannot see; a flag set once at boot would be stale. So every enable re-checks GCONF
// and rewrites the current while EN is still high, and only then pulls EN low.
void cmd_tmc_enable(bool on)
{
  if (!on) {
    driver_off();
    reply_text(true, "tmc en off");
    return;
  }
  if (g_motor.enabled) {
    reply_text(true, "tmc en on (already)");
    return;
  }
  if (!g_motor.have_current) {
    reply_text(false, "tmc: write IHOLD_IRUN (0x10) first");
    return;
  }
  uint32_t gconf = 0;
  jb::tmc2209::ReplyStatus why = jb::tmc2209::ReplyStatus::Ok;
  jb::TmcStatus status = jb::tmc_read(jb::tmc2209::kGconf, &gconf, &why);
  if (status != jb::TmcStatus::Ok) {
    reply_tmc_failure(status, why);
    return;
  }
  if (const char * refusal = jb::tmc2209::bench_gconf_refusal(gconf)) {
    reply_text(false, refusal);
    return;
  }
  if ((gconf & jb::tmc2209::kGconfPdnDisable) == 0) {
    reply_text(false, "tmc: set GCONF pdn_disable (bit 6) first");
    return;
  }
  // At IRUN 16 the current is below StealthChop's automatic-tuning minimum (datasheet p.42), so
  // the bench runs SpreadCycle. The SPREAD pin is unconnected on this module and read low in
  // IOIN at step 1; a high SPREAD pin would invert this bit (p.23).
  if ((gconf & jb::tmc2209::kGconfEnSpreadCycle) == 0) {
    reply_text(false, "tmc: set GCONF en_SpreadCycle (bit 2) first, the bench runs SpreadCycle");
    return;
  }
  uint32_t ioin = 0;
  status = jb::tmc_read(jb::tmc2209::kIoin, &ioin, &why);
  if (status != jb::TmcStatus::Ok) {
    reply_tmc_failure(status, why);
    return;
  }
  if ((ioin & jb::tmc2209::kIoinSpreadEn) != 0) {
    reply_text(false, "tmc: SPREAD pin reads high, which would invert the chopper choice");
    return;
  }
  // Clear the reset and drv_err records (write 1 to clear), then confirm nothing is left: a
  // fault that is still present, or uv_cp with VM low, keeps the driver off.
  uint32_t before = 0;
  uint32_t after = 0;
  bool counted = false;
  status = counted_write(jb::tmc2209::kGstat, jb::tmc2209::kGstatAll, &before, &after, &counted,
      &why);
  if (status != jb::TmcStatus::Ok) {
    reply_tmc_failure(status, why);
    return;
  }
  if (!counted) {
    reply_text(false, "tmc: GSTAT clear not counted, driver left off");
    return;
  }
  uint32_t gstat = 0;
  status = jb::tmc_read(jb::tmc2209::kGstat, &gstat, &why);
  if (status != jb::TmcStatus::Ok) {
    reply_tmc_failure(status, why);
    return;
  }
  if ((gstat & jb::tmc2209::kGstatAll) != 0) {
    char buf[jb::kMaxLine];
    jb::TextWriter w(buf, sizeof(buf));
    w.str("tmc: GSTAT still flags a fault, driver left off gstat=");
    hex32(w, gstat);
    reply_text(false, std::string_view(buf, w.size()));
    return;
  }
  status = counted_write(jb::tmc2209::kIholdIrun, g_motor.ihold_irun, &before, &after, &counted,
      &why);
  if (status != jb::TmcStatus::Ok) {
    reply_tmc_failure(status, why);
    return;
  }
  if (!counted) {
    reply_text(false, "tmc: IHOLD_IRUN rewrite not counted, driver left off");
    return;
  }
  HAL_GPIO_WritePin(TMC_EN_GPIO_Port, TMC_EN_Pin, GPIO_PIN_RESET);
  g_motor.enabled = true;
  g_motor.gconf = gconf;
  g_motor.last_check_us = jb::now_us();
  g_motor.check_phase = 0;
  char buf[jb::kMaxLine];
  jb::TextWriter w(buf, sizeof(buf));
  w.str("OK tmc en on gconf=");
  hex32(w, gconf);
  w.str(" ihold_irun=");
  hex32(w, g_motor.ihold_irun);
  w.put('\n');
  send(w, buf);
}

// "OK move n=1600 dir=+ period_us=2500"; "OK move done n=1600" follows when it ends.
void cmd_move(const jb::MoveArgs & m)
{
  if (!g_motor.enabled) {
    reply_text(false, "move: enable the driver first (tmc en on)");
    return;
  }
  if (g_motor.gen.active()) {
    reply_text(false, "move: already moving (stop first)");
    return;
  }
  // The in-move GCONF check blocks for about 1.2 ms. It runs right after an edge, so it fits
  // while the period is at least 2 ms; faster stepping waits for timer PWM in stage C.
  if (m.rate_hz > kMaxSupervisedRateHz) {
    reply_text(false, "move: rate above 500 Hz is not supervised yet (stage C)");
    return;
  }
  const bool reverse = m.steps < 0;
  const uint32_t count =
    static_cast<uint32_t>(reverse ? -static_cast<int64_t>(m.steps) : m.steps);
  const uint32_t period = 1000000u / m.rate_hz;
  HAL_GPIO_WritePin(TMC_DIR_GPIO_Port, TMC_DIR_Pin, reverse ? GPIO_PIN_SET : GPIO_PIN_RESET);
  busy_wait_us(kDirSetupUs);
  g_motor.gen.start(count, period, jb::now_us());
  char buf[jb::kMaxLine];
  jb::TextWriter w(buf, sizeof(buf));
  w.str("OK move n=");
  w.u64(count);
  w.str(reverse ? " dir=-" : " dir=+");
  w.str(" period_us=");
  w.u64(period);
  w.put('\n');
  send(w, buf);
}

void reply_move_end(bool ok, const char * what)
{
  char buf[jb::kMaxLine];
  jb::TextWriter w(buf, sizeof(buf));
  w.str(ok ? "OK " : "ERR ");
  w.str(what);
  w.str(" n=");
  w.u64(g_motor.gen.issued());
  w.put('\n');
  send(w, buf);
}

// Called on every pass of the main loop while the driver is enabled, moving or not.
void supervise()
{
  if (!g_motor.enabled) {
    return;
  }
  const uint64_t now = jb::now_us();
  if (g_motor.check_phase == 0) {
    if (now - g_motor.last_check_us < kCheckIntervalUs) {
      return;
    }
    g_motor.last_check_us = now;
    g_motor.check_phase = 1;
  }
  // Never let a read push the next edge late.
  if (g_motor.gen.active() && g_motor.gen.next_due_us() < now + kCheckBudgetUs) {
    return;
  }
  const bool gstat_turn = g_motor.check_phase == 1;
  const uint8_t reg = gstat_turn ? jb::tmc2209::kGstat : jb::tmc2209::kGconf;
  uint32_t value = 0;
  jb::tmc2209::ReplyStatus why = jb::tmc2209::ReplyStatus::Ok;
  const jb::TmcStatus status = jb::tmc_read(reg, &value, &why);
  if (status != jb::TmcStatus::Ok) {
    reply_tmc_failure(status, why);  // switches the driver off
    return;
  }
  const bool bad = gstat_turn ? (value & jb::tmc2209::kGstatAll) != 0 : value != g_motor.gconf;
  if (bad) {
    driver_off();
    char buf[jb::kMaxLine];
    jb::TextWriter w(buf, sizeof(buf));
    w.str(gstat_turn ? "ERR tmc: GSTAT fault, driver off gstat=" :
      "ERR tmc: GCONF changed (driver reset?), driver off gconf=");
    hex32(w, value);
    w.put('\n');
    send(w, buf);
    return;
  }
  g_motor.check_phase = gstat_turn ? 2 : 0;
}

// Called on every pass of the main loop. A due edge is raised, held for kStepHighUs and dropped
// here, so STEP is never left high.
void motor_poll()
{
  switch (g_motor.gen.poll(jb::now_us())) {
    case jb::StepGen::Action::None:
      return;
    case jb::StepGen::Action::Step:
      HAL_GPIO_WritePin(TMC_STEP_GPIO_Port, TMC_STEP_Pin, GPIO_PIN_SET);
      busy_wait_us(kStepHighUs);
      HAL_GPIO_WritePin(TMC_STEP_GPIO_Port, TMC_STEP_Pin, GPIO_PIN_RESET);
      return;
    case jb::StepGen::Action::Done:
      reply_move_end(true, "move done");
      return;
    case jb::StepGen::Action::Late:
      driver_off();
      reply_move_end(false, "move: a step came late, driver off");
      return;
  }
}

// --- Endstop 1 (B10), polled from the main loop ---
// The level is reported as read: which level means "flag in the slot" depends on the sensor
// module and is settled on the bench, then handled by homing.
jb::Debouncer g_es1(2000);  // 2 ms, inside the wiring guide's 1-5 ms
bool g_es1_watch = false;

bool es1_raw()
{
  return HAL_GPIO_ReadPin(ES1_GPIO_Port, ES1_Pin) == GPIO_PIN_SET;
}

// With watch on, every accepted change is sent as an event line, "...,es1:1" or "...,es1:0",
// timestamped like mark.
void endstop_poll()
{
  const uint64_t now = jb::now_us();
  if (!g_es1.feed(es1_raw(), now) || !g_es1_watch) {
    return;
  }
  char buf[jb::kMaxLine];
  const uint32_t len =
    jb::format_event(buf, sizeof(buf), now, g_es1.level() ? "es1:1" : "es1:0");
  if (len) {
    jb::usb_send(buf, len);
  }
}

// "OK es level=1 raw=1 changes=3 raw_edges=5 watch=off"
void cmd_endstop_read()
{
  char buf[jb::kMaxLine];
  jb::TextWriter w(buf, sizeof(buf));
  w.str("OK es level=");
  w.u64(g_es1.level() ? 1 : 0);
  w.str(" raw=");
  w.u64(es1_raw() ? 1 : 0);
  w.str(" changes=");
  w.u64(g_es1.changes());
  w.str(" raw_edges=");
  w.u64(g_es1.raw_edges());
  w.str(g_es1_watch ? " watch=on" : " watch=off");
  w.put('\n');
  send(w, buf);
}

void cmd_stats()
{
  const jb::ImuCounters c = jb::imu_counters();
  char buf[jb::kMaxLine * 2];
  jb::TextWriter w(buf, sizeof(buf));
  w.str("OK stats samples=");
  w.u64(c.samples);
  w.str(" imu_overrun=");
  w.u64(c.overrun);
  w.str(" i2c_err=");
  w.u64(c.i2c_err);
  w.str(" queue_drop=");
  w.u64(c.queue_drop);
  w.str(" tx_drop=");
  w.u64(jb::usb_tx_dropped());
  w.str(" rx_drop=");
  w.u64(jb::usb_rx_dropped());
  w.put('\n');
  send(w, buf);
}

void handle(const jb::Command & cmd)
{
  const bool streaming = jb::imu_streaming();
  // Anything that blocks for milliseconds would hold up the STEP edges, which are paced from
  // this loop. Only commands that return at once run during a move.
  if (g_motor.gen.active()) {
    switch (cmd.kind) {
      case jb::CmdKind::Help:
      case jb::CmdKind::Ver:
      case jb::CmdKind::Stats:
      case jb::CmdKind::Mark:
      case jb::CmdKind::EndstopRead:
      case jb::CmdKind::EndstopWatch:
      case jb::CmdKind::StreamOff:
      case jb::CmdKind::Stop:
      case jb::CmdKind::Invalid:
        break;
      case jb::CmdKind::TmcEnable:
        if (!cmd.tmc.on) {break;}
        reply_text(false, "busy: moving (stop first)");
        return;
      default:
        reply_text(false, "busy: moving (stop first)");
        return;
    }
  }
  // With the driver enabled, the GSTAT/GCONF checks must run every 100 ms, so IMU commands that
  // can block for that long (bus scan, probe, reconfigure with its 100 ms settle) wait until
  // the driver is off.
  if (g_motor.enabled) {
    switch (cmd.kind) {
      case jb::CmdKind::I2cScan:
      case jb::CmdKind::ImuInfo:
      case jb::CmdKind::ImuCfg:
      case jb::CmdKind::StreamOn:
        reply_text(false, "busy: driver enabled (tmc en off first)");
        return;
      default:
        break;
    }
  }
  switch (cmd.kind) {
    case jb::CmdKind::Help:
      reply_text(true, "help ver i2cscan stats | imu info | imu cfg rate= dlpf= fs= | "
        "stream on|off | mark [label] | tmc read <reg> | tmc write <reg> <val> | "
        "tmc en on|off | move <usteps> <hz> | stop | es [watch on|off]");
      break;
    case jb::CmdKind::Ver:
      reply_text(true, "ver joint_bench " JB_VERSION);
      break;
    case jb::CmdKind::I2cScan:
      // Scanning and reconfiguring use the bus in blocking mode, which would collide with
      // the DMA reads that streaming starts from the interrupt.
      if (streaming) {reply_text(false, "stop the stream first"); break;}
      cmd_i2cscan();
      break;
    case jb::CmdKind::ImuInfo:
      if (ensure_imu()) {reply_imu_info();}
      break;
    case jb::CmdKind::ImuCfg:
      if (streaming) {reply_text(false, "stop the stream first"); break;}
      cmd_imu_cfg(cmd.cfg);
      break;
    case jb::CmdKind::StreamOn:
      if (ensure_imu()) {
        jb::imu_stream(true);
        reply_text(true, "stream on");
      }
      break;
    case jb::CmdKind::StreamOff:
      jb::imu_stream(false);
      reply_text(true, "stream off");
      break;
    case jb::CmdKind::Mark:
      cmd_mark(cmd.label);
      break;
    case jb::CmdKind::Stats:
      cmd_stats();
      break;
    // TMC transactions block for a few milliseconds, long enough to back up the sample queue.
    case jb::CmdKind::TmcRead:
      if (streaming) {reply_text(false, "stop the stream first"); break;}
      cmd_tmc_read(cmd.tmc.reg);
      break;
    case jb::CmdKind::TmcWrite:
      if (streaming) {reply_text(false, "stop the stream first"); break;}
      cmd_tmc_write(cmd.tmc.reg, cmd.tmc.value);
      break;
    case jb::CmdKind::TmcEnable:
      if (cmd.tmc.on && streaming) {reply_text(false, "stop the stream first"); break;}
      cmd_tmc_enable(cmd.tmc.on);
      break;
    case jb::CmdKind::Move:
      if (streaming) {reply_text(false, "stop the stream first"); break;}
      cmd_move(cmd.move);
      break;
    case jb::CmdKind::Stop:
      if (g_motor.gen.active()) {
        g_motor.gen.stop();
        reply_move_end(true, "stop");
      } else {
        reply_text(true, "stop (idle)");
      }
      break;
    case jb::CmdKind::EndstopRead:
      cmd_endstop_read();
      break;
    case jb::CmdKind::EndstopWatch:
      g_es1_watch = cmd.tmc.on;
      reply_text(true, g_es1_watch ? "es watch on" : "es watch off");
      break;
    case jb::CmdKind::Invalid:
      reply_text(false, cmd.error);
      break;
  }
}

void blink()
{
  // Slow blink: running with an IMU. Fast blink: no IMU found. Steady: the loop is stuck.
  const uint32_t period = jb::imu_present() ? 500 : 100;
  if (HAL_GetTick() - g_last_blink >= period) {
    g_last_blink = HAL_GetTick();
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
}

}  // namespace

extern "C" void app_init(void)
{
  jb::clock_init();
  g_es1.reset(es1_raw(), jb::now_us());
  jb::imu_probe(g_cfg);  // failure is fine here; see ensure_imu()
}

extern "C" void app_loop(void)
{
  motor_poll();
  supervise();
  endstop_poll();
  jb::now_us();  // keeps the 64-bit clock extension fed even when nothing else is running
  char c;
  while (jb::usb_rx_pop(&c)) {
    switch (g_rx.feed(c)) {
      case jb::LineAssembler::Result::Line: handle(jb::parse_command(g_rx.line())); break;
      case jb::LineAssembler::Result::Overflow: reply_text(false, "line too long"); break;
      case jb::LineAssembler::Result::None: break;
    }
  }
  jb::ImuSample s;
  char buf[jb::kMaxLine];
  // Bounded, so a burst of samples cannot starve command handling or the USB pump.
  for (int i = 0; i < 16 && jb::imu_pop(&s); ++i) {
    const uint32_t len = jb::format_sample(buf, sizeof(buf), s);
    if (len) {
      jb::usb_send(buf, len);
    }
  }
  jb::usb_pump();
  blink();
}

extern "C" void app_usb_rx(const uint8_t * buf, uint32_t len)
{
  jb::usb_rx_isr(buf, len);
}

extern "C" void app_usb_tx_done(void)
{
  jb::usb_tx_done_isr();
}
