#pragma once

#include <cstdint>
#include <string_view>

namespace jb
{

enum class CmdKind
{
  Invalid,
  Help,
  Ver,
  I2cScan,
  ImuInfo,
  ImuCfg,
  StreamOn,
  StreamOff,
  Mark,
  Stats,
  TmcRead,
  TmcWrite,
  TmcEnable,
  Move,
  Stop,
  EndstopRead,
  EndstopWatch,
};

// Fields left at 0 keep their current value. Whether a value is one the chip supports is
// the IMU driver's call, not the parser's: the parser only knows it is a number.
struct ImuCfgArgs
{
  uint16_t rate_hz = 0;
  uint16_t dlpf_hz = 0;
  uint8_t fs_g = 0;
};

// Range checks only. Which registers may be written is the TMC policy's call
// (jb::tmc2209::bench_write_refusal), not the parser's.
struct TmcArgs
{
  uint8_t reg = 0;     // 0..127
  uint32_t value = 0;  // tmc write only
  bool on = false;     // tmc en, es watch
};

// Bench limits: 64000 microsteps is 40 turns at 1/8; 2000 Hz is 1.25 turns a second.
constexpr int32_t kMaxMoveSteps = 64000;
constexpr uint32_t kMaxMoveRateHz = 2000;

struct MoveArgs
{
  int32_t steps = 0;     // microsteps, sign = direction, never 0
  uint32_t rate_hz = 0;  // STEP rising edges per second, 1..kMaxMoveRateHz
};

struct Command
{
  CmdKind kind = CmdKind::Invalid;
  ImuCfgArgs cfg;
  TmcArgs tmc;
  MoveArgs move;
  char label[24] {};  // mark only; NUL-terminated, empty when no label was given
  const char * error = nullptr;  // set when kind is Invalid
};

// Parses one line from the console. Words are separated by one or more spaces.
//
//   help | ver | i2cscan | stats
//   imu info
//   imu cfg [rate=<Hz>] [dlpf=<Hz>] [fs=<g>]   at least one key
//   stream on | stream off
//   mark [label]                               label: 1-23 chars of [A-Za-z0-9_-]
//   tmc read <reg>                             reg: 0-127
//   tmc write <reg> <value>                    value: 0-0xFFFFFFFF
//   tmc en on | tmc en off
//   move <microsteps> <rate_hz>                microsteps: +-1..kMaxMoveSteps, not 0
//   stop
//   es | es watch on | es watch off
//
// tmc numbers are decimal, or hex with a 0x prefix, because datasheets list registers in hex.
//
// The label is restricted because it is written into a comma-separated event line; a
// comma in it would shift every field the host parses after it.
Command parse_command(std::string_view line);

}  // namespace jb
