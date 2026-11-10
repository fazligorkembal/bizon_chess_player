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

// Register addresses used by the bench (TMC2209 datasheet Rev. 1.09, section 5).
constexpr uint8_t kGconf = 0x00;
constexpr uint8_t kGstat = 0x01;
constexpr uint8_t kIfcnt = 0x02;
constexpr uint8_t kIoin = 0x06;
constexpr uint8_t kIholdIrun = 0x10;  // write-only

// GSTAT bits (p.24). reset and drv_err are cleared by writing 1; uv_cp is not latched.
constexpr uint32_t kGstatReset = 1u << 0;
constexpr uint32_t kGstatDrvErr = 1u << 1;
constexpr uint32_t kGstatUvCp = 1u << 2;
constexpr uint32_t kGstatAll = kGstatReset | kGstatDrvErr | kGstatUvCp;

// IOIN bits (p.25).
constexpr uint32_t kIoinSpreadEn = 1u << 8;

// GCONF bits (p.23).
constexpr uint32_t kGconfIScaleAnalog = 1u << 0;
constexpr uint32_t kGconfInternalRsense = 1u << 1;
constexpr uint32_t kGconfEnSpreadCycle = 1u << 2;
constexpr uint32_t kGconfShaft = 1u << 3;
constexpr uint32_t kGconfPdnDisable = 1u << 6;
constexpr uint32_t kGconfMstepRegSelect = 1u << 7;
constexpr uint32_t kGconfTestMode = 1u << 9;

// The highest IRUN the bench accepts. With R110 sense resistors and VREF trimmed to 0.50 V,
// IRUN 16 is about 0.19 A RMS; the pot, not this number, is the hard ceiling.
constexpr uint32_t kBenchMaxIrun = 16;

// Bench write policy. Returns nullptr when `tmc write` may send this register and value, or
// the reason it may not. Two registers are open:
// - GCONF, only in the bench profile: I_scale_analog set, so the VREF pot keeps capping the
//   current; internal_Rsense, shaft, mstep_reg_select and test_mode clear; no reserved bits.
//   Clearing I_scale_analog would take the pot out and multiply the current several times.
// - IHOLD_IRUN, with IRUN <= kBenchMaxIrun, IHOLD <= IRUN and no reserved bits.
// Everything else stays closed; the OTP programming register in particular, because OTP writes
// cannot be undone.
const char * bench_write_refusal(uint8_t register_address, uint32_t value);

// The GCONF part of the policy on its own, for checking a value read back from the driver.
const char * bench_gconf_refusal(uint32_t value);

const char * reply_status_name(ReplyStatus status);

bool make_read(uint8_t address, uint8_t register_address, ReadRequest & out);
bool make_write(uint8_t address, uint8_t register_address, uint32_t value, Datagram & out);
ReplyStatus decode_reply(
  const uint8_t * bytes, size_t length, uint8_t expected_register, uint32_t & out);

}
