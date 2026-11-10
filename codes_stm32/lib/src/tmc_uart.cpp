#include "jb/tmc_uart.hpp"

namespace jb::tmc2209
{
namespace
{

uint8_t checksum(const uint8_t * bytes, size_t length)
{
  uint8_t crc = 0;
  for (size_t index = 0; index < length; ++index) {
    uint8_t current = bytes[index];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      const bool feedback = ((crc >> 7) ^ (current & 1)) != 0;
      crc = static_cast<uint8_t>(crc << 1);
      if (feedback) {
        crc ^= 0x07;
      }
      current >>= 1;
    }
  }
  return crc;
}

}

const char * bench_gconf_refusal(uint32_t value)
{
  // Bits 0-9 are defined (p.23); anything above is reserved.
  constexpr uint32_t kDefined = (1u << 10) - 1;
  if ((value & ~kDefined) != 0) {
    return "tmc: GCONF reserved bits must stay 0";
  }
  if ((value & kGconfTestMode) != 0) {
    return "tmc: GCONF test_mode (bit 9) must stay 0";
  }
  if ((value & kGconfIScaleAnalog) == 0) {
    return "tmc: GCONF I_scale_analog (bit 0) must stay 1, so VREF caps the current";
  }
  if ((value & kGconfInternalRsense) != 0) {
    return "tmc: GCONF internal_Rsense (bit 1) must stay 0, the module has sense resistors";
  }
  if ((value & (kGconfShaft | kGconfMstepRegSelect)) != 0) {
    return "tmc: GCONF shaft (bit 3) and mstep_reg_select (bit 7) stay 0 on the bench";
  }
  return nullptr;
}

const char * bench_write_refusal(uint8_t register_address, uint32_t value)
{
  if (register_address == kGconf) {
    return bench_gconf_refusal(value);
  }
  if (register_address == kIholdIrun) {
    // IHOLD [4:0], IRUN [12:8], IHOLDDELAY [19:16] (p.28).
    constexpr uint32_t kDefined = 0x000F1F1F;
    if ((value & ~kDefined) != 0) {
      return "tmc: IHOLD_IRUN reserved bits must stay 0";
    }
    const uint32_t ihold = value & 0x1F;
    const uint32_t irun = (value >> 8) & 0x1F;
    if (irun > kBenchMaxIrun) {
      return "tmc: IRUN above the bench limit of 16";
    }
    if (ihold > irun) {
      return "tmc: IHOLD must not exceed IRUN";
    }
    return nullptr;
  }
  return "tmc: only GCONF (0x00) and IHOLD_IRUN (0x10) may be written on the bench";
}

const char * reply_status_name(ReplyStatus status)
{
  switch (status) {
    case ReplyStatus::Ok: return "ok";
    case ReplyStatus::InvalidLength: return "length";
    case ReplyStatus::InvalidSync: return "sync";
    case ReplyStatus::InvalidMasterAddress: return "master address";
    case ReplyStatus::UnexpectedRegister: return "register";
    case ReplyStatus::CrcMismatch: return "crc";
  }
  return "unknown";
}

bool make_read(uint8_t address, uint8_t register_address, ReadRequest & out)
{
  if (address > 3 || register_address > 0x7F) {
    return false;
  }
  ReadRequest request{0x05, address, register_address, 0};
  request.back() = checksum(request.data(), request.size() - 1);
  out = request;
  return true;
}

bool make_write(uint8_t address, uint8_t register_address, uint32_t value, Datagram & out)
{
  if (address > 3 || register_address > 0x7F) {
    return false;
  }
  Datagram request{
    0x05, address, static_cast<uint8_t>(register_address | 0x80),
    static_cast<uint8_t>(value >> 24), static_cast<uint8_t>(value >> 16),
    static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value), 0};
  request.back() = checksum(request.data(), request.size() - 1);
  out = request;
  return true;
}

ReplyStatus decode_reply(
  const uint8_t * bytes, size_t length, uint8_t expected_register, uint32_t & out)
{
  if (bytes == nullptr || length != Datagram{}.size()) {
    return ReplyStatus::InvalidLength;
  }
  if (bytes[0] != 0x05) {
    return ReplyStatus::InvalidSync;
  }
  if (bytes[1] != 0xFF) {
    return ReplyStatus::InvalidMasterAddress;
  }
  if (expected_register > 0x7F || bytes[2] != expected_register) {
    return ReplyStatus::UnexpectedRegister;
  }
  if (checksum(bytes, length - 1) != bytes[length - 1]) {
    return ReplyStatus::CrcMismatch;
  }
  out = (static_cast<uint32_t>(bytes[3]) << 24) |
    (static_cast<uint32_t>(bytes[4]) << 16) |
    (static_cast<uint32_t>(bytes[5]) << 8) |
    static_cast<uint32_t>(bytes[6]);
  return ReplyStatus::Ok;
}

}
