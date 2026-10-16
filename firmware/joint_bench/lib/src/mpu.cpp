#include "jb/mpu.hpp"

#include <cstddef>

namespace jb::mpu
{
namespace
{

struct Option
{
  uint8_t code;
  uint16_t hz;
};

// DLPF_CFG 0 and 7 are excluded on purpose: on both families they switch the gyro to an
// 8 kHz internal rate, which changes what SMPLRT_DIV divides and breaks the rate maths.
constexpr Option kMpu6050Dlpf[] = {{1, 184}, {2, 94}, {3, 44}, {4, 21}, {5, 10}, {6, 5}};
constexpr Option kMpu6500Gyro[] = {{1, 184}, {2, 92}, {3, 41}, {4, 20}, {5, 10}, {6, 5}};
constexpr Option kMpu6500Accel[] = {{1, 218}, {2, 99}, {3, 45}, {4, 21}, {5, 10}, {6, 5}};

template<std::size_t N>
const Option & nearest(const Option (&options)[N], uint16_t hz)
{
  const Option * best = &options[0];
  for (const Option & o : options) {
    const int d = o.hz > hz ? o.hz - hz : hz - o.hz;
    const int bd = best->hz > hz ? best->hz - hz : hz - best->hz;
    if (d < bd) {
      best = &o;
    }
  }
  return *best;
}

}  // namespace

Chip identify(uint8_t who_am_i)
{
  switch (who_am_i) {
    case 0x68: return Chip::Mpu6050;
    case 0x70:
    case 0x71: return Chip::Mpu6500;
    default: return Chip::Unknown;
  }
}

const char * chip_name(Chip chip)
{
  switch (chip) {
    case Chip::Mpu6050: return "MPU6050";
    case Chip::Mpu6500: return "MPU6500";
    default: return "unknown";
  }
}

const char * plan_config(Chip chip, const Config & want, Plan * out)
{
  if (chip == Chip::Unknown) {
    return "unknown chip";
  }
  // With the DLPF on, the output rate is 1 kHz / (1 + SMPLRT_DIV), divider 0..255.
  if (want.rate_hz == 0 || want.rate_hz > 1000 || 1000 % want.rate_hz != 0 ||
    1000 / want.rate_hz > 256)
  {
    return "rate must divide 1000 Hz";
  }
  uint8_t fs_bits = 0;
  uint16_t lsb = 0;
  switch (want.fs_g) {
    case 2: fs_bits = 0u << 3; lsb = 16384; break;
    case 4: fs_bits = 1u << 3; lsb = 8192; break;
    case 8: fs_bits = 2u << 3; lsb = 4096; break;
    case 16: fs_bits = 3u << 3; lsb = 2048; break;
    default: return "fs must be 2, 4, 8 or 16";
  }
  if (want.dlpf_hz == 0) {
    return "dlpf must be positive";
  }
  Plan p{};
  auto add = [&p](uint8_t r, uint8_t v) {p.writes[p.count++] = RegWrite{r, v};};
  // CLKSEL = 1: the X-gyro PLL, far more accurate than the internal 8 MHz oscillator,
  // and the sample interval follows it.
  add(reg::kPwrMgmt1, 0x01);
  add(reg::kSmplrtDiv, static_cast<uint8_t>(1000 / want.rate_hz - 1));
  uint16_t accel_hz = 0;
  if (chip == Chip::Mpu6050) {
    // One DLPF setting serves both sensors. EXT_SYNC_SET stays 0, so FSYNC is ignored.
    const Option & o = nearest(kMpu6050Dlpf, want.dlpf_hz);
    add(reg::kConfig, o.code);
    accel_hz = o.hz;
  } else {
    const Option & g = nearest(kMpu6500Gyro, want.dlpf_hz);
    const Option & a = nearest(kMpu6500Accel, want.dlpf_hz);
    add(reg::kConfig, g.code);
    accel_hz = a.hz;
  }
  add(reg::kGyroConfig, 1u << 3);  // +-500 dps; streamed, but no metric uses it
  add(reg::kAccelConfig, fs_bits);
  if (chip == Chip::Mpu6500) {
    add(reg::kAccelConfig2, nearest(kMpu6500Accel, want.dlpf_hz).code);
  }
  // Active-high push-pull 50 us pulse per sample (LATCH_INT_EN = 0), status cleared by
  // any read (INT_RD_CLEAR = 1). A pulse cannot get stuck high if a read is ever missed.
  add(reg::kIntPinCfg, 0x10);
  add(reg::kIntEnable, 0x01);  // DATA_RDY_EN
  p.accepted = Config{want.rate_hz, accel_hz, want.fs_g};
  p.lsb_per_g = lsb;
  *out = p;
  return nullptr;
}

Raw parse_block(const uint8_t * b)
{
  auto be16 = [b](int i) {
      return static_cast<int16_t>(static_cast<uint16_t>(b[i] << 8 | b[i + 1]));
    };
  return Raw{{be16(0), be16(2), be16(4)}, be16(6), {be16(8), be16(10), be16(12)}};
}

}  // namespace jb::mpu
