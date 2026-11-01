#include "app.h"

#include <string_view>

#include "clock.hpp"
#include "imu.hpp"
#include "jb/command.hpp"
#include "jb/line_assembler.hpp"
#include "jb/telemetry.hpp"
#include "jb/text_writer.hpp"
#include "main.h"
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
  char buf[128];
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
  switch (cmd.kind) {
    case jb::CmdKind::Help:
      reply_text(true, "help ver i2cscan stats | imu info | imu cfg rate= dlpf= fs= | "
        "stream on|off | mark [label]");
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
  jb::imu_probe(g_cfg);  // failure is fine here; see ensure_imu()
}

extern "C" void app_loop(void)
{
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
