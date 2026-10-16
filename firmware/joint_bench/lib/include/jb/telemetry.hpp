#pragma once

#include <cstdint>
#include <string_view>

namespace jb
{

// One IMU reading as it leaves the interrupt: raw counts, no scaling. The scale travels
// once, in the `imu info` reply, rather than in every line.
struct ImuSample
{
  uint32_t seq;
  uint64_t t_us;
  int16_t accel[3];
  int16_t gyro[3];
};

// Longest possible line is 76 bytes ("T," + 10 + "," + 20 + 6 x 7 + "\n"); 96 leaves room.
constexpr uint32_t kMaxLine = 96;

// "T,<seq>,<t_us>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>\n". Returns the length, or 0 if it does
// not fit in cap.
uint32_t format_sample(char * buf, uint32_t cap, const ImuSample & s);

// "E,<t_us>,<name>\n". Returns the length, or 0 if it does not fit in cap.
uint32_t format_event(char * buf, uint32_t cap, uint64_t t_us, std::string_view name);

}  // namespace jb
