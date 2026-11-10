#include "tmc_bus.hpp"

#include "clock.hpp"
#include "main.h"
#include "usart.h"

namespace jb
{
namespace
{

// MS1 and MS2 are strapped to GND on the bench.
constexpr uint8_t kNode = 0;

// At 115200 baud a byte is 87 us, so eight bytes plus the default SENDDELAY of eight bit times
// take under 1 ms. HAL_GetTick counts whole milliseconds, so allow a few.
constexpr uint32_t kSendTimeoutMs = 5;
constexpr uint32_t kReplyTimeoutMs = 5;

// After an error the TMC2209 needs at least 12 bit times of idle line before it accepts a new
// datagram (datasheet Rev. 1.09, section 4.1.1). 1 ms is about 115 bit times.
void recover_after_error()
{
  HAL_Delay(1);
}

// The TMC2209 keeps driving the line for four bit times after the last stop bit of its reply
// (datasheet Rev. 1.09, p.20, section 4.3), about 35 us at 115200 baud. A request sent straight
// after a reply, as `tmc write` does after reading IFCNT, must not start inside that window.
// Whether the USART's idle frame on enabling TE already covers it would need RM0368, which is
// not at hand, so wait explicitly: 100 us is about 11 bit times.
constexpr uint64_t kTurnaroundUs = 100;

void wait_turnaround()
{
  const uint64_t start = now_us();
  while (now_us() - start < kTurnaroundUs) {
  }
}

bool send(const uint8_t * bytes, uint16_t length)
{
  wait_turnaround();
  // With the receiver off while sending, the USART does not hear its own bytes on the shared
  // line, so the reply that follows is not preceded by an echo of the request.
  if (HAL_HalfDuplex_EnableTransmitter(&huart1) != HAL_OK) {
    return false;
  }
  // HAL_UART_Transmit returns only after the last stop bit has left (TC), so switching to the
  // receiver straight after it cannot cut the request short.
  return HAL_UART_Transmit(&huart1, const_cast<uint8_t *>(bytes), length, kSendTimeoutMs) ==
         HAL_OK;
}

void drop_stale_input()
{
  // Anything left in the data register, or an overrun from a previous failed exchange, would
  // otherwise be taken as the first byte of the next reply.
  __HAL_UART_CLEAR_OREFLAG(&huart1);
  while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
    (void)huart1.Instance->DR;
  }
}

}  // namespace

TmcStatus tmc_read(uint8_t register_address, uint32_t * value, tmc2209::ReplyStatus * why)
{
  tmc2209::ReadRequest request{};
  if (!tmc2209::make_read(kNode, register_address, request)) {
    return TmcStatus::BadArgument;
  }
  if (!send(request.data(), static_cast<uint16_t>(request.size()))) {
    recover_after_error();
    return TmcStatus::SendFailed;
  }
  tmc2209::Datagram reply{};
  drop_stale_input();
  if (HAL_HalfDuplex_EnableReceiver(&huart1) != HAL_OK ||
    HAL_UART_Receive(&huart1, reply.data(), static_cast<uint16_t>(reply.size()),
      kReplyTimeoutMs) != HAL_OK)
  {
    recover_after_error();
    return TmcStatus::NoReply;
  }
  const tmc2209::ReplyStatus status =
    tmc2209::decode_reply(reply.data(), reply.size(), register_address, *value);
  if (why != nullptr) {
    *why = status;
  }
  if (status != tmc2209::ReplyStatus::Ok) {
    recover_after_error();
    return TmcStatus::BadReply;
  }
  return TmcStatus::Ok;
}

TmcStatus tmc_write(uint8_t register_address, uint32_t value)
{
  tmc2209::Datagram request{};
  if (!tmc2209::make_write(kNode, register_address, value, request)) {
    return TmcStatus::BadArgument;
  }
  if (!send(request.data(), static_cast<uint16_t>(request.size()))) {
    recover_after_error();
    return TmcStatus::SendFailed;
  }
  // A write gets no reply. Only IFCNT shows whether the driver accepted it; the caller checks.
  return TmcStatus::Ok;
}

}  // namespace jb
