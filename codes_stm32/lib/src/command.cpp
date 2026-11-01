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
  } else {
    return invalid("unknown command");
  }
  if (n != 1) {
    return invalid("this command takes no arguments");
  }
  return kind_only(kind);
}

}  // namespace jb
