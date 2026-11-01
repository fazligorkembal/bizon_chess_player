#pragma once

#include <cstdint>

namespace jb
{

// Queues one whole line for the host. Returns false, and counts the drop, when the
// transmit ring is full; it never waits, because a stalled host must not stall sampling.
bool usb_send(const char * line, uint32_t len);

// Starts the next USB transfer when the previous one has finished. Main loop only.
void usb_pump();

// Pops one received byte. Main loop only.
bool usb_rx_pop(char * c);

uint32_t usb_tx_dropped();
uint32_t usb_rx_dropped();

// Interrupt-side hooks, called through app.h from usbd_cdc_if.c.
void usb_rx_isr(const uint8_t * buf, uint32_t len);
void usb_tx_done_isr();

}  // namespace jb
