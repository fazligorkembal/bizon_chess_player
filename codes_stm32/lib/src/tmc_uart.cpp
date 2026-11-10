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

const char * bench_write_refusal(uint8_t register_address, uint32_t value)
{
  constexpr uint32_t kGconfTestMode = 1u << 9;
  if (register_address != kGconf) {
    return "tmc: only GCONF (0x00) may be written on the bench";
  }
  if ((value & kGconfTestMode) != 0) {
    return "tmc: GCONF test_mode (bit 9) must stay 0";
  }
  return nullptr;
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
