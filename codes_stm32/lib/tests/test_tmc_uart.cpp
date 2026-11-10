#include <gtest/gtest.h>

#include "jb/tmc_uart.hpp"

namespace jb::tmc2209
{

void PrintTo(ReplyStatus status, std::ostream * stream)
{
  switch (status) {
    case ReplyStatus::Ok: *stream << "Ok"; break;
    case ReplyStatus::InvalidLength: *stream << "InvalidLength"; break;
    case ReplyStatus::InvalidSync: *stream << "InvalidSync"; break;
    case ReplyStatus::InvalidMasterAddress: *stream << "InvalidMasterAddress"; break;
    case ReplyStatus::UnexpectedRegister: *stream << "UnexpectedRegister"; break;
    case ReplyStatus::CrcMismatch: *stream << "CrcMismatch"; break;
  }
}

}

namespace
{

using jb::tmc2209::Datagram;
using jb::tmc2209::ReadRequest;
using jb::tmc2209::ReplyStatus;

constexpr Datagram kIoinReply{0x05, 0xFF, 0x06, 0x21, 0x00, 0x00, 0x00, 0x41};

}

TEST(TmcUart, BuildsKnownReadRequestsIncludingTheirCrc)
{
  ReadRequest request{};
  ASSERT_TRUE(jb::tmc2209::make_read(0, 0x00, request));
  EXPECT_EQ(request, (ReadRequest{0x05, 0x00, 0x00, 0x48}));
  ASSERT_TRUE(jb::tmc2209::make_read(0, 0x06, request));
  EXPECT_EQ(request, (ReadRequest{0x05, 0x00, 0x06, 0x6F}));
}

TEST(TmcUart, BuildsKnownWriteWithBigEndianDataAndWriteBit)
{
  Datagram request{};
  ASSERT_TRUE(jb::tmc2209::make_write(0, 0x10, 0x12345678, request));
  EXPECT_EQ(request, (Datagram{0x05, 0x00, 0x90, 0x12, 0x34, 0x56, 0x78, 0xEF}));
}

TEST(TmcUart, SupportsAllFourNodeAddressesAndBothRegisterBoundaries)
{
  for (uint8_t address = 0; address < 4; ++address) {
    for (const uint8_t register_address : {0x00, 0x7F}) {
      ReadRequest read{};
      Datagram write{};
      ASSERT_TRUE(jb::tmc2209::make_read(address, register_address, read));
      ASSERT_TRUE(jb::tmc2209::make_write(address, register_address, 0xFFFFFFFF, write));
      EXPECT_EQ(read[1], address);
      EXPECT_EQ(write[1], address);
      EXPECT_EQ(read[2], register_address);
      EXPECT_EQ(write[2], register_address | 0x80);
      for (size_t index = 3; index < 7; ++index) {
        EXPECT_EQ(write[index], 0xFF);
      }
    }
  }
}

TEST(TmcUart, RejectsInvalidAddressesWithoutModifyingRequests)
{
  const ReadRequest original_read{1, 2, 3, 4};
  const Datagram original_write{1, 2, 3, 4, 5, 6, 7, 8};
  auto read = original_read;
  auto write = original_write;
  for (const uint8_t address : {4, 0xFF}) {
    EXPECT_FALSE(jb::tmc2209::make_read(address, 0, read));
    EXPECT_FALSE(jb::tmc2209::make_write(address, 0, 0, write));
  }
  for (const uint8_t register_address : {0x80, 0xFF}) {
    EXPECT_FALSE(jb::tmc2209::make_read(0, register_address, read));
    EXPECT_FALSE(jb::tmc2209::make_write(0, register_address, 0, write));
  }
  EXPECT_EQ(read, original_read);
  EXPECT_EQ(write, original_write);
}

TEST(TmcUart, DecodesIoinReplyAddressedToMaster)
{
  uint32_t value = 0;
  ASSERT_EQ(
    jb::tmc2209::decode_reply(kIoinReply.data(), kIoinReply.size(), 0x06, value),
    ReplyStatus::Ok);
  EXPECT_EQ(value, 0x21000000u);
}

TEST(TmcUart, RejectsShortLongAndNullRepliesWithoutChangingOutput)
{
  uint32_t value = 0xDEADBEEF;
  for (size_t length = 0; length <= 9; ++length) {
    if (length == kIoinReply.size()) {
      continue;
    }
    EXPECT_EQ(
      jb::tmc2209::decode_reply(kIoinReply.data(), length, 0x06, value),
      ReplyStatus::InvalidLength);
  }
  EXPECT_EQ(jb::tmc2209::decode_reply(nullptr, 8, 0x06, value), ReplyStatus::InvalidLength);
  EXPECT_EQ(value, 0xDEADBEEFu);
}

TEST(TmcUart, RejectsEverySingleBitCorruptionWithoutChangingOutput)
{
  for (size_t byte_index = 0; byte_index < kIoinReply.size(); ++byte_index) {
    for (uint8_t bit = 0; bit < 8; ++bit) {
      auto reply = kIoinReply;
      reply[byte_index] ^= static_cast<uint8_t>(1u << bit);
      uint32_t value = 0xDEADBEEF;
      EXPECT_NE(
        jb::tmc2209::decode_reply(reply.data(), reply.size(), 0x06, value), ReplyStatus::Ok);
      EXPECT_EQ(value, 0xDEADBEEFu);
    }
  }
}

TEST(TmcUart, DistinguishesHeaderRegisterAndChecksumFailures)
{
  uint32_t value = 0xDEADBEEF;
  auto reply = kIoinReply;
  reply[0] = 0x04;
  EXPECT_EQ(
    jb::tmc2209::decode_reply(reply.data(), reply.size(), 0x06, value),
    ReplyStatus::InvalidSync);
  reply = kIoinReply;
  reply[1] = 0;
  EXPECT_EQ(
    jb::tmc2209::decode_reply(reply.data(), reply.size(), 0x06, value),
    ReplyStatus::InvalidMasterAddress);
  for (const uint8_t register_address : {0x02, 0x80, 0xFF}) {
    EXPECT_EQ(
      jb::tmc2209::decode_reply(kIoinReply.data(), kIoinReply.size(), register_address, value),
      ReplyStatus::UnexpectedRegister);
  }
  reply = kIoinReply;
  reply[2] |= 0x80;
  EXPECT_EQ(
    jb::tmc2209::decode_reply(reply.data(), reply.size(), 0x06, value),
    ReplyStatus::UnexpectedRegister);
  reply = kIoinReply;
  reply.back() ^= 1;
  EXPECT_EQ(
    jb::tmc2209::decode_reply(reply.data(), reply.size(), 0x06, value),
    ReplyStatus::CrcMismatch);
  EXPECT_EQ(value, 0xDEADBEEFu);
}

TEST(TmcUart, DoesNotAcceptAnEchoedWriteAsAReply)
{
  Datagram request{};
  ASSERT_TRUE(jb::tmc2209::make_write(0, 0x06, 0x21000000, request));
  uint32_t value = 0xDEADBEEF;
  EXPECT_EQ(
    jb::tmc2209::decode_reply(request.data(), request.size(), 0x06, value),
    ReplyStatus::InvalidMasterAddress);
  EXPECT_EQ(value, 0xDEADBEEFu);
}

TEST(TmcUart, BenchPolicyOpensOnlyGconfAndNeverItsTestModeBit)
{
  EXPECT_EQ(jb::tmc2209::bench_write_refusal(jb::tmc2209::kGconf, 0x000001C1), nullptr);
  EXPECT_NE(jb::tmc2209::bench_write_refusal(jb::tmc2209::kGconf, 1u << 9), nullptr);
  EXPECT_NE(jb::tmc2209::bench_write_refusal(jb::tmc2209::kGconf, 0xFFFFFFFF), nullptr);
  for (uint8_t reg = 1; reg < 0x80; ++reg) {
    EXPECT_NE(jb::tmc2209::bench_write_refusal(reg, 0), nullptr) << int(reg);
  }
}
