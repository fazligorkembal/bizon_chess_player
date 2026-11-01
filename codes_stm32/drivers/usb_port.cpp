#include "usb_port.hpp"

#include "jb/line_ring.hpp"
#include "jb/spsc_queue.hpp"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

namespace jb
{
namespace
{
// 8 KB holds about 160 ms of telemetry at 1 kHz: enough to ride out a host that is busy
// for a moment, small enough that a host that has stopped reading is noticed quickly.
LineRing<8192> g_tx;
SpscQueue<char, 256> g_rx;
uint32_t g_inflight = 0;      // bytes handed to the USB stack and not yet confirmed
volatile bool g_tx_done = false;
volatile uint32_t g_rx_dropped = 0;

bool configured()
{
  // CDC_Transmit_FS dereferences the class data unchecked, and that pointer is null until
  // the host has enumerated the device. Sending before then is a hard fault, so check.
  return hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED && hUsbDeviceFS.pClassData != nullptr;
}
}  // namespace

bool usb_send(const char * line, uint32_t len)
{
  return g_tx.push(line, len);
}

void usb_pump()
{
  if (!configured()) {
    // Unplugged or not yet enumerated. A transfer that was in flight will never complete,
    // and waiting for it would silence the port for good after a replug. Drop it.
    if (g_inflight != 0) {
      g_tx.consume(g_inflight);
      g_inflight = 0;
    }
    return;
  }
  if (g_inflight != 0) {
    if (!g_tx_done) {
      return;
    }
    // Only now may the ring reuse those bytes: the stack read them in place.
    g_tx.consume(g_inflight);
    g_inflight = 0;
  }
  const uint8_t * data = nullptr;
  uint32_t len = g_tx.peek(&data);
  if (len == 0) {
    return;
  }
  if (len > 4096) {
    len = 4096;  // CDC_Transmit_FS takes a uint16_t length
  }
  g_tx_done = false;
  if (CDC_Transmit_FS(const_cast<uint8_t *>(data), static_cast<uint16_t>(len)) == USBD_OK) {
    g_inflight = len;
  }
}

bool usb_rx_pop(char * c)
{
  return g_rx.pop(c);
}

uint32_t usb_tx_dropped() {return g_tx.dropped();}
uint32_t usb_rx_dropped() {return g_rx_dropped;}

void usb_rx_isr(const uint8_t * buf, uint32_t len)
{
  for (uint32_t i = 0; i < len; ++i) {
    if (!g_rx.push(static_cast<char>(buf[i]))) {
      g_rx_dropped = g_rx_dropped + 1;
    }
  }
}

void usb_tx_done_isr()
{
  g_tx_done = true;
}

}  // namespace jb
