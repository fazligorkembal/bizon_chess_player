#include "clock.hpp"

#include "jb/cycle_clock.hpp"
#include "main.h"

namespace jb
{
namespace
{
CycleClock g_clock(84000000);  // SYSCLK; see the clock tree in the design doc, section 4.2
}

void clock_init()
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint64_t now_us()
{
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const uint64_t cycles = g_clock.extend(DWT->CYCCNT);
  __set_PRIMASK(primask);
  return g_clock.to_us(cycles);
}

}  // namespace jb
