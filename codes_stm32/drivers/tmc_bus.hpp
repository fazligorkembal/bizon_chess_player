#pragma once

#include <cstdint>

#include "jb/tmc_uart.hpp"

namespace jb
{

enum class TmcStatus : uint8_t
{
  Ok,
  BadArgument,  // register or node out of range; nothing was sent
  SendFailed,   // the USART did not finish sending
  NoReply,      // fewer than eight bytes came back before the deadline
  BadReply,     // eight bytes came back but failed validation; see `why`
};

// Blocking single-wire transactions with the TMC2209 at node 0 on USART1 (PA9, half-duplex).
// Each call takes a few milliseconds, so never call them while the IMU is streaming.
//
// The driver answers only when VM is powered: its logic runs from the internal regulator on VS,
// not from VCC_IO. A NoReply with VM off is expected, not a fault.
TmcStatus tmc_read(uint8_t register_address, uint32_t * value, tmc2209::ReplyStatus * why);
TmcStatus tmc_write(uint8_t register_address, uint32_t value);

}  // namespace jb
