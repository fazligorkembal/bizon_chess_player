// Placeholder application: proves the CubeMX seam builds, flashes and runs before any bench
// logic exists. The heartbeat at 2 Hz is the whole of its behaviour. Task 11 replaces it.
#include "app.h"

#include "main.h"

extern "C" void app_init(void) {}

extern "C" void app_loop(void)
{
  static uint32_t last = 0;
  if (HAL_GetTick() - last >= 250) {
    last = HAL_GetTick();
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
}

extern "C" void app_usb_rx(const uint8_t * buf, uint32_t len)
{
  (void)buf;
  (void)len;
}

extern "C" void app_usb_tx_done(void) {}
