#include "jb/command.hpp"

#include <cstddef>

namespace jb
{
namespace
{

constexpr int kMaxWords = 6;

// string_view::substr and ::copy throw std::out_of_range on a bad index; even with
// -fno-exceptions that drags libstdc++'s throw helpers into the image. Slicing by pointer
// cannot fail, so this file never calls them.
std::string_view slice(std::string_view s, size_t pos, size_t len)
{
  return std::string_view(s.data() + pos, len);
}

int split(std::string_view line, std::string_view * words)
{
  int n = 0;
  size_t i = 0;
  while (i < line.size()) {
    while (i < line.size() && line[i] == ' ') {
      ++i;
    }
    if (i == line.size()) {
      break;
    }
    size_t j = i;
    while (j < line.size() && line[j] != ' ') {
      ++j;
    }
    if (n == kMaxWords) {
      return kMaxWords + 1;
    }
    words[n++] = slice(line, i, j - i);
    i = j;
  }
  return n;
}

// Decimal digits only, 1..max. Anything else, including a sign or an overflow, is false.
bool parse_uint(std::string_view text, uint32_t max, uint32_t * out)
{
  if (text.empty() || text.size() > 5) {
    return false;
  }
  uint32_t value = 0;
  for (char c : text) {
    if (c < '0' || c > '9') {
      return false;
    }
    value = value * 10 + static_cast<uint32_t>(c - '0');
  }
  if (value == 0 || value > max) {
    return false;
  }
  *out = value;
  return true;
}

// Like parse_uint, but zero is allowed and "0x" selects hex: 0..max, at most 32 bits.
bool parse_number(std::string_view text, uint32_t max, uint32_t * out)
{
  uint64_t value = 0;
  if (text.size() > 2 && text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) {
    if (text.size() > 10) {
      return false;
    }
    for (size_t i = 2; i < text.size(); ++i) {
      const char c = text[i];
      uint32_t digit = 0;
      if (c >= '0' && c <= '9') {
        digit = static_cast<uint32_t>(c - '0');
      } else if (c >= 'a' && c <= 'f') {
        digit = static_cast<uint32_t>(c - 'a' + 10);
      } else if (c >= 'A' && c <= 'F') {
        digit = static_cast<uint32_t>(c - 'A' + 10);
      } else {
        return false;
      }
      value = value * 16 + digit;
    }
  } else {
    if (text.empty() || text.size() > 10) {
      return false;
    }
    for (char c : text) {
      if (c < '0' || c > '9') {
        return false;
      }
      value = value * 10 + static_cast<uint32_t>(c - '0');
    }
  }
  if (value > max) {
    return false;
  }
  *out = static_cast<uint32_t>(value);
  return true;
}

bool label_char(char c)
{
  return (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
         c == '_' || c == '-';
}

Command invalid(const char * error)
{
  Command cmd;
  cmd.error = error;
  return cmd;
}

Command kind_only(CmdKind kind)
{
  Command cmd;
  cmd.kind = kind;
  return cmd;
}

Command parse_cfg(const std::string_view * words, int n)
{
  if (n == 2) {
    return invalid("cfg: nothing to set");
  }
  Command cmd;
  cmd.kind = CmdKind::ImuCfg;
  for (int i = 2; i < n; ++i) {
    const size_t eq = words[i].find('=');
    if (eq == std::string_view::npos) {
      return invalid("cfg: expected key=value");
    }
    const std::string_view key = slice(words[i], 0, eq);
    const std::string_view val = slice(words[i], eq + 1, words[i].size() - eq - 1);
    uint32_t v = 0;
    if (key == "rate") {
      if (!parse_uint(val, 65535, &v)) {return invalid("cfg: bad value for rate");}
      cmd.cfg.rate_hz = static_cast<uint16_t>(v);
    } else if (key == "dlpf") {
      if (!parse_uint(val, 65535, &v)) {return invalid("cfg: bad value for dlpf");}
      cmd.cfg.dlpf_hz = static_cast<uint16_t>(v);
    } else if (key == "fs") {
      if (!parse_uint(val, 255, &v)) {return invalid("cfg: bad value for fs");}
      cmd.cfg.fs_g = static_cast<uint8_t>(v);
    } else {
      return invalid("cfg: unknown key");
    }
  }
  return cmd;
}

Command parse_mark(const std::string_view * words, int n)
{
  Command cmd;
  cmd.kind = CmdKind::Mark;
  if (n == 1) {
    return cmd;
  }
  if (n > 2) {
    return invalid("mark: one label at most");
  }
  const std::string_view label = words[1];
  if (label.size() >= sizeof(cmd.label)) {
    return invalid("mark: label too long");
  }
  for (char c : label) {
    if (!label_char(c)) {
      return invalid("mark: label must be [A-Za-z0-9_-]");
    }
  }
  for (size_t i = 0; i < label.size(); ++i) {
    cmd.label[i] = label[i];
  }
  cmd.label[label.size()] = '\0';
  return cmd;
}

// Optional minus sign, then decimal digits; magnitude 1..max.
bool parse_signed(std::string_view text, int32_t max, int32_t * out)
{
  bool negative = false;
  if (!text.empty() && text[0] == '-') {
    negative = true;
    text = slice(text, 1, text.size() - 1);
  }
  uint32_t magnitude = 0;
  if (!parse_uint(text, static_cast<uint32_t>(max), &magnitude)) {
    return false;
  }
  *out = negative ? -static_cast<int32_t>(magnitude) : static_cast<int32_t>(magnitude);
  return true;
}

Command parse_move(const std::string_view * words, int n)
{
  if (n != 3) {
    return invalid("move: expected <microsteps> <rate_hz>");
  }
  Command cmd;
  cmd.kind = CmdKind::Move;
  if (!parse_signed(words[1], kMaxMoveSteps, &cmd.move.steps)) {
    return invalid("move: microsteps must be +-1..64000");
  }
  if (!parse_uint(words[2], kMaxMoveRateHz, &cmd.move.rate_hz)) {
    return invalid("move: rate must be 1..2000 Hz");
  }
  return cmd;
}

Command parse_tmc(const std::string_view * words, int n)
{
  Command cmd;
  uint32_t reg = 0;
  if (n >= 2 && words[1] == "en") {
    if (n == 3 && (words[2] == "on" || words[2] == "off")) {
      cmd.kind = CmdKind::TmcEnable;
      cmd.tmc.on = words[2] == "on";
      return cmd;
    }
    return invalid("tmc en: expected on or off");
  }
  if (n >= 2 && words[1] == "read") {
    if (n != 3) {return invalid("tmc read: expected a register");}
    cmd.kind = CmdKind::TmcRead;
  } else if (n >= 2 && words[1] == "write") {
    if (n != 4) {return invalid("tmc write: expected a register and a value");}
    cmd.kind = CmdKind::TmcWrite;
    if (!parse_number(words[3], 0xFFFFFFFFu, &cmd.tmc.value)) {
      return invalid("tmc: bad value");
    }
  } else {
    return invalid("tmc: expected read, write or en");
  }
  if (!parse_number(words[2], 0x7F, &reg)) {
    return invalid("tmc: register must be 0-127");
  }
  cmd.tmc.reg = static_cast<uint8_t>(reg);
  return cmd;
}

}  // namespace

Command parse_command(std::string_view line)
{
  std::string_view words[kMaxWords];
  const int n = split(line, words);
  if (n == 0) {
    return invalid("empty line");
  }
  if (n > kMaxWords) {
    return invalid("too many words");
  }
  const std::string_view head = words[0];
  if (head == "mark") {
    return parse_mark(words, n);
  }
  if (head == "imu") {
    if (n >= 2 && words[1] == "cfg") {
      return parse_cfg(words, n);
    }
    if (n == 2 && words[1] == "info") {
      return kind_only(CmdKind::ImuInfo);
    }
    return invalid("imu: expected info or cfg");
  }
  if (head == "tmc") {
    return parse_tmc(words, n);
  }
  if (head == "move") {
    return parse_move(words, n);
  }
  if (head == "es") {
    if (n == 1) {
      return kind_only(CmdKind::EndstopRead);
    }
    if (n == 3 && words[1] == "watch" && (words[2] == "on" || words[2] == "off")) {
      Command cmd;
      cmd.kind = CmdKind::EndstopWatch;
      cmd.tmc.on = words[2] == "on";
      return cmd;
    }
    return invalid("es: expected nothing, or watch on|off");
  }
  if (head == "stream") {
    if (n == 2 && words[1] == "on") {
      return kind_only(CmdKind::StreamOn);
    }
    if (n == 2 && words[1] == "off") {
      return kind_only(CmdKind::StreamOff);
    }
    return invalid("stream: expected on or off");
  }
  CmdKind kind = CmdKind::Invalid;
  if (head == "help") {
    kind = CmdKind::Help;
  } else if (head == "ver") {
    kind = CmdKind::Ver;
  } else if (head == "i2cscan") {
    kind = CmdKind::I2cScan;
  } else if (head == "stats") {
    kind = CmdKind::Stats;
  } else if (head == "stop") {
    kind = CmdKind::Stop;
  } else {
    return invalid("unknown command");
  }
  if (n != 1) {
    return invalid("this command takes no arguments");
  }
  return kind_only(kind);
}

}  // namespace jb
