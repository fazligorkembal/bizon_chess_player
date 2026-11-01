#pragma once

#include <cstdint>

// Register-level logic for the InvenSense MPU-6050 (GY-521) and the MPU-6500 family
// (MPU-6500, MPU-9250), kept free of I/O so it is testable on the host. The driver in
// drivers/ performs the writes this module plans.
namespace jb::mpu
{

constexpr uint8_t kAddrLow = 0x68;   // AD0 low, the GY-521 default
constexpr uint8_t kAddrHigh = 0x69;  // AD0 high

namespace reg
{
constexpr uint8_t kSmplrtDiv = 0x19;
constexpr uint8_t kConfig = 0x1A;
constexpr uint8_t kGyroConfig = 0x1B;
constexpr uint8_t kAccelConfig = 0x1C;
constexpr uint8_t kAccelConfig2 = 0x1D;  // MPU-6500 family only
constexpr uint8_t kIntPinCfg = 0x37;
constexpr uint8_t kIntEnable = 0x38;
constexpr uint8_t kAccelXoutH = 0x3B;  // start of the 14-byte data block
constexpr uint8_t kPwrMgmt1 = 0x6B;
constexpr uint8_t kWhoAmI = 0x75;
}  // namespace reg

constexpr uint8_t kResetValue = 0x80;  // PWR_MGMT_1.DEVICE_RESET; wait 100 ms after it
constexpr uint8_t kBlockLen = 14;      // accel xyz, temperature, gyro xyz, big-endian

enum class Chip : uint8_t { Unknown, Mpu6050, Mpu6500 };

// WHO_AM_I: 0x68 MPU-6050, 0x70 MPU-6500, 0x71 MPU-9250 (an MPU-6500 core plus a
// magnetometer). Clone parts answer other values; those are reported, not guessed at.
Chip identify(uint8_t who_am_i);
const char * chip_name(Chip chip);

struct Config
{
  uint16_t rate_hz;  // output data rate
  uint16_t dlpf_hz;  // accelerometer low-pass bandwidth; the nearest supported is used
  uint8_t fs_g;      // accelerometer full scale: 2, 4, 8 or 16
};

constexpr Config kDefault{1000, 188, 4};

struct RegWrite
{
  uint8_t reg;
  uint8_t value;
};

struct Plan
{
  RegWrite writes[8];
  uint8_t count;
  Config accepted;      // what the chip will actually do, echoed back to the user
  uint16_t lsb_per_g;   // raw accelerometer counts per g at the accepted full scale
};

// Plans the register writes that put the chip in `want`, to be applied after a reset.
// Returns nullptr on success, or a message naming the value that cannot be met.
const char * plan_config(Chip chip, const Config & want, Plan * out);

struct Raw
{
  int16_t accel[3];
  int16_t temp;
  int16_t gyro[3];
};

// Decodes the 14-byte block read from ACCEL_XOUT_H.
Raw parse_block(const uint8_t * block);

}  // namespace jb::mpu
