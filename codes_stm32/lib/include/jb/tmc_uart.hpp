#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace jb::tmc2209
{

using ReadRequest = std::array<uint8_t, 4>;
using Datagram = std::array<uint8_t, 8>;

enum class ReplyStatus : uint8_t
{
  Ok,
  InvalidLength,
  InvalidSync,
  InvalidMasterAddress,
  UnexpectedRegister,
  CrcMismatch,
};

// Register addresses used by the bench (TMC2209 datasheet Rev. 1.09, section 5.1).
constexpr uint8_t kGconf = 0x00;
constexpr uint8_t kIfcnt = 0x02;

// Bench write policy. Returns nullptr when `tmc write` may send this register and value, or
// the reason it may not. Only GCONF is open while the bus is being brought up, and never with
// test_mode (GCONF bit 9) set. Each later register is opened deliberately, once its effect on
// the motor is understood; the OTP programming register in particular stays closed, because
// OTP writes cannot be undone.
const char * bench_write_refusal(uint8_t register_address, uint32_t value);

const char * reply_status_name(ReplyStatus status);

bool make_read(uint8_t address, uint8_t register_address, ReadRequest & out);
bool make_write(uint8_t address, uint8_t register_address, uint32_t value, Datagram & out);
ReplyStatus decode_reply(
  const uint8_t * bytes, size_t length, uint8_t expected_register, uint32_t & out);

}
