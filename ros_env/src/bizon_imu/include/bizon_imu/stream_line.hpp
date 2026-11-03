#pragma once

#include <array>
#include <charconv>
#include <cstdint>
#include <optional>
#include <string_view>
#include <system_error>

namespace bizon_imu
{

// One `T,<seq>,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>` line of the joint_bench firmware's stream,
// in raw sensor counts. tools/joint_bench/joint_bench/protocol.py reads the same format.
struct Sample
{
  uint32_t seq{};
  uint32_t t_us{};
  std::array<int, 3> accel{};
  std::array<int, 3> gyro{};
};

namespace detail
{
template<typename T>
bool take_number(std::string_view & text, T & value)
{
  const char * const end = text.data() + text.size();
  const auto [ptr, ec] = std::from_chars(text.data(), end, value);
  if (ec != std::errc{}) {
    return false;
  }
  text.remove_prefix(static_cast<std::size_t>(ptr - text.data()));
  return true;
}

template<typename T>
bool take_field(std::string_view & text, T & value)
{
  if (text.empty() || text.front() != ',') {
    return false;
  }
  text.remove_prefix(1);
  return take_number(text, value);
}
}  // namespace detail

// Nullopt for anything but a complete sample line. The first read after opening the port can
// start mid-line, and a truncated number would otherwise parse as a plausible wrong value, so
// trailing text and missing fields both reject the line.
inline std::optional<Sample> parse_sample(std::string_view line)
{
  if (line.empty() || line.front() != 'T') {
    return std::nullopt;
  }
  line.remove_prefix(1);
  Sample s;
  bool ok = detail::take_field(line, s.seq) && detail::take_field(line, s.t_us);
  for (int & v : s.accel) {
    ok = ok && detail::take_field(line, v);
  }
  for (int & v : s.gyro) {
    ok = ok && detail::take_field(line, v);
  }
  if (!ok || !line.empty()) {
    return std::nullopt;
  }
  return s;
}

// Reads `lsb_per_g=<n>` from the reply to `imu info`. The accelerometer's scale comes from the
// firmware because it depends on the range the firmware configured.
inline std::optional<int> parse_lsb_per_g(std::string_view reply)
{
  constexpr std::string_view key = "lsb_per_g=";
  const auto pos = reply.find(key);
  if (pos == std::string_view::npos) {
    return std::nullopt;
  }
  reply.remove_prefix(pos + key.size());
  int value = 0;
  if (!detail::take_number(reply, value) || value <= 0) {
    return std::nullopt;
  }
  return value;
}

}  // namespace bizon_imu
