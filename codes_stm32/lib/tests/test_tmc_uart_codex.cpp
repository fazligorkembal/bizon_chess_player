#include <gtest/gtest.h>

#include "jb/tmc_uart.hpp"

namespace
{

using jb::tmc2209::Datagram;
using jb::tmc2209::ReadRequest;
using jb::tmc2209::ReplyStatus;

// All CRC literals below were calculated separately with the C-code example in
// TMC2209 datasheet Rev. 1.09 (2023-FEB-16), p.20, section 4.2. No production
// checksum or packet builder is used to construct the expected replies.
constexpr Datagram kIoinReply{0x05, 0xFF, 0x06, 0x21, 0x00, 0x00, 0x00, 0x41};
constexpr uint32_t kUnchanged = 0xDEADBEEF;

}  // namespace

TEST(TmcUartCodex, ReadCrcCoversEveryNodeAddressAndTheRequestedRegister)
{
  // Section 4.1.2, p.19: node 0..3, read bit clear, four bytes including CRC.
  const ReadRequest ifcnt_reads[]{
    {0x05, 0x00, 0x02, 0x8F},
    {0x05, 0x01, 0x02, 0x39},
    {0x05, 0x02, 0x02, 0xD4},
    {0x05, 0x03, 0x02, 0x62},
  };
  const ReadRequest ioin_reads[]{
    {0x05, 0x00, 0x06, 0x6F},
    {0x05, 0x01, 0x06, 0xD9},
    {0x05, 0x02, 0x06, 0x34},
    {0x05, 0x03, 0x06, 0x82},
  };
  for (uint8_t node = 0; node < 4; ++node) {
    ReadRequest request{};
    ASSERT_TRUE(jb::tmc2209::make_read(node, 0x02, request));
    EXPECT_EQ(request, ifcnt_reads[node]);
    ASSERT_TRUE(jb::tmc2209::make_read(node, 0x06, request));
    EXPECT_EQ(request, ioin_reads[node]);
  }
}

TEST(TmcUartCodex, WriteCrcCoversEveryNodeAddressAndZeroDataBytes)
{
  // Section 4.1.1, p.18: bit 7 marks write; all seven preceding bytes enter CRC.
  // This is a packet vector, not a motor-configuration recommendation.
  const Datagram writes[]{
    {0x05, 0x00, 0x80, 0x00, 0x00, 0x01, 0x40, 0xF1},
    {0x05, 0x01, 0x80, 0x00, 0x00, 0x01, 0x40, 0x1D},
    {0x05, 0x02, 0x80, 0x00, 0x00, 0x01, 0x40, 0x87},
    {0x05, 0x03, 0x80, 0x00, 0x00, 0x01, 0x40, 0x6B},
  };
  for (uint8_t node = 0; node < 4; ++node) {
    Datagram request{};
    ASSERT_TRUE(jb::tmc2209::make_write(node, 0x00, 0x00000140, request));
    EXPECT_EQ(request, writes[node]);
  }
}

TEST(TmcUartCodex, DecodesAllFourDataBytesInBigEndianOrderWithBit31Set)
{
  // Section 4.1.2, p.19: data bytes 3,2,1,0 (high to low byte).
  // Synthetic raw payload; no register-field semantics are asserted here.
  constexpr Datagram reply{0x05, 0xFF, 0x6F, 0x89, 0xAB, 0xCD, 0xEF, 0xD2};
  uint32_t value = 0;
  ASSERT_EQ(
    jb::tmc2209::decode_reply(reply.data(), reply.size(), 0x6F, value), ReplyStatus::Ok);
  EXPECT_EQ(value, 0x89ABCDEFu);
}

TEST(TmcUartCodex, RejectsEachReservedReplyBitEvenWithACorrectCrc)
{
  // Reply diagram, section 4.1.2, p.19: reserved (0), unlike request don't-cares.
  const Datagram replies[]{
    {0x15, 0xFF, 0x06, 0x21, 0x00, 0x00, 0x00, 0xAB},
    {0x25, 0xFF, 0x06, 0x21, 0x00, 0x00, 0x00, 0x34},
    {0x45, 0xFF, 0x06, 0x21, 0x00, 0x00, 0x00, 0xF8},
    {0x85, 0xFF, 0x06, 0x21, 0x00, 0x00, 0x00, 0x9E},
  };
  for (const auto & reply : replies) {
    uint32_t value = kUnchanged;
    EXPECT_EQ(
      jb::tmc2209::decode_reply(reply.data(), reply.size(), 0x06, value),
      ReplyStatus::InvalidSync);
    EXPECT_EQ(value, kUnchanged);
  }
}

TEST(TmcUartCodex, RejectsWrongSyncAndMasterAddressEvenWithACorrectCrc)
{
  // Section 4.1.2, p.19: reply sync 0x05 and master address 0xFF.
  constexpr Datagram wrong_sync{0x04, 0xFF, 0x06, 0x21, 0x00, 0x00, 0x00, 0xCB};
  uint32_t value = kUnchanged;
  EXPECT_EQ(
    jb::tmc2209::decode_reply(wrong_sync.data(), wrong_sync.size(), 0x06, value),
    ReplyStatus::InvalidSync);
  EXPECT_EQ(value, kUnchanged);

  const Datagram replies[]{
    {0x05, 0x00, 0x06, 0x21, 0x00, 0x00, 0x00, 0xEE},
    {0x05, 0x03, 0x06, 0x21, 0x00, 0x00, 0x00, 0x74},
    {0x05, 0xFE, 0x06, 0x21, 0x00, 0x00, 0x00, 0xAD},
  };
  for (const auto & reply : replies) {
    value = kUnchanged;
    EXPECT_EQ(
      jb::tmc2209::decode_reply(reply.data(), reply.size(), 0x06, value),
      ReplyStatus::InvalidMasterAddress);
    EXPECT_EQ(value, kUnchanged);
  }
}

TEST(TmcUartCodex, RejectsWrongRegisterAndReplyWriteBitEvenWithACorrectCrc)
{
  // Section 4.1.2, p.19: reply register must match the request, with RW=0.
  const Datagram replies[]{
    {0x05, 0xFF, 0x02, 0x21, 0x00, 0x00, 0x00, 0x25},
    {0x05, 0xFF, 0x86, 0x21, 0x00, 0x00, 0x00, 0x23},
  };
  for (const auto & reply : replies) {
    uint32_t value = kUnchanged;
    EXPECT_EQ(
      jb::tmc2209::decode_reply(reply.data(), reply.size(), 0x06, value),
      ReplyStatus::UnexpectedRegister);
    EXPECT_EQ(value, kUnchanged);
  }
}

TEST(TmcUartCodex, RequiresTransportToSeparateReadEchoFromTheEightByteReply)
{
  // Section 4.4, p.21: all transmitted data is also received by RXD.
  // The documented decoder boundary is one reply, after the four-byte echo.
  constexpr std::array<uint8_t, 12> received{
    0x05, 0x00, 0x06, 0x6F,
    0x05, 0xFF, 0x06, 0x21, 0x00, 0x00, 0x00, 0x41};
  uint32_t value = kUnchanged;
  EXPECT_EQ(
    jb::tmc2209::decode_reply(received.data(), received.size(), 0x06, value),
    ReplyStatus::InvalidLength);
  EXPECT_EQ(value, kUnchanged);
  EXPECT_EQ(
    jb::tmc2209::decode_reply(received.data(), ReadRequest{}.size(), 0x06, value),
    ReplyStatus::InvalidLength);
  EXPECT_EQ(value, kUnchanged);
  EXPECT_EQ(
    jb::tmc2209::decode_reply(received.data(), Datagram{}.size(), 0x06, value),
    ReplyStatus::InvalidMasterAddress);
  EXPECT_EQ(value, kUnchanged);
  ASSERT_EQ(
    jb::tmc2209::decode_reply(received.data() + ReadRequest{}.size(),
      kIoinReply.size(), 0x06, value), ReplyStatus::Ok);
  EXPECT_EQ(value, 0x21000000u);
}

TEST(TmcUartCodex, DecodesIfcntValuesOnBothSidesOfItsEightBitWrap)
{
  // Section 5.1, p.24: IFCNT is eight bits and wraps from 255 to 0.
  // The transaction layer, not decode_reply, must compare the modulo-256 delta.
  constexpr Datagram before{0x05, 0xFF, 0x02, 0x00, 0x00, 0x00, 0xFF, 0xBF};
  constexpr Datagram after{0x05, 0xFF, 0x02, 0x00, 0x00, 0x00, 0x00, 0x4C};
  uint32_t value = kUnchanged;
  ASSERT_EQ(
    jb::tmc2209::decode_reply(before.data(), before.size(), 0x02, value), ReplyStatus::Ok);
  EXPECT_EQ(value, 255u);
  ASSERT_EQ(
    jb::tmc2209::decode_reply(after.data(), after.size(), 0x02, value), ReplyStatus::Ok);
  EXPECT_EQ(value, 0u);
}
