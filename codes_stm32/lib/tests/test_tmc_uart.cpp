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

TEST(TmcUart, BenchPolicyKeepsGconfInTheBenchProfile)
{
  using jb::tmc2209::bench_write_refusal;
  using jb::tmc2209::kGconf;
  EXPECT_EQ(bench_write_refusal(kGconf, 0x00000145), nullptr);  // the first-motion profile
  EXPECT_EQ(bench_write_refusal(kGconf, 0x00000141), nullptr);
  EXPECT_NE(bench_write_refusal(kGconf, 0x00000144), nullptr);  // I_scale_analog cleared
  EXPECT_NE(bench_write_refusal(kGconf, 0x00000147), nullptr);  // internal_Rsense
  EXPECT_NE(bench_write_refusal(kGconf, 0x0000014D), nullptr);  // shaft
  EXPECT_NE(bench_write_refusal(kGconf, 0x000001C5), nullptr);  // mstep_reg_select
  EXPECT_NE(bench_write_refusal(kGconf, 0x00000345), nullptr);  // test_mode
  EXPECT_NE(bench_write_refusal(kGconf, 0x00000545), nullptr);  // reserved bit 10
  EXPECT_EQ(jb::tmc2209::bench_gconf_refusal(0x00000145), nullptr);
  EXPECT_NE(jb::tmc2209::bench_gconf_refusal(0x00000144), nullptr);
}

TEST(TmcUart, BenchPolicyCapsIholdIrun)
{
  using jb::tmc2209::bench_write_refusal;
  using jb::tmc2209::kIholdIrun;
  EXPECT_EQ(bench_write_refusal(kIholdIrun, 0x00011008), nullptr);  // IHOLD 8, IRUN 16, delay 1
  EXPECT_EQ(bench_write_refusal(kIholdIrun, 0x00000000), nullptr);
  EXPECT_EQ(bench_write_refusal(kIholdIrun, 0x000F1010), nullptr);  // IHOLD = IRUN = 16
  EXPECT_NE(bench_write_refusal(kIholdIrun, 0x00001108), nullptr);  // IRUN 17
  EXPECT_NE(bench_write_refusal(kIholdIrun, 0x00001F00), nullptr);  // IRUN 31, the reset value
  EXPECT_NE(bench_write_refusal(kIholdIrun, 0x00000809), nullptr);  // IHOLD 9 > IRUN 8
  EXPECT_NE(bench_write_refusal(kIholdIrun, 0x00100808), nullptr);  // reserved bit 20
  EXPECT_NE(bench_write_refusal(kIholdIrun, 0x00002008), nullptr);  // reserved bit 13
}

TEST(TmcUart, BenchPolicyKeepsEveryOtherRegisterClosed)
{
  for (unsigned reg = 0; reg < 0x80; ++reg) {
    if (reg == jb::tmc2209::kGconf || reg == jb::tmc2209::kIholdIrun) {
      continue;
    }
    EXPECT_NE(jb::tmc2209::bench_write_refusal(static_cast<uint8_t>(reg), 0), nullptr) << reg;
  }
}
