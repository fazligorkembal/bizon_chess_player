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
};

struct Command
{
  CmdKind kind = CmdKind::Invalid;
  ImuCfgArgs cfg;
  TmcArgs tmc;
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
//
// tmc numbers are decimal, or hex with a 0x prefix, because datasheets list registers in hex.
//
// The label is restricted because it is written into a comma-separated event line; a
// comma in it would shift every field the host parses after it.
Command parse_command(std::string_view line);

}  // namespace jb
