#pragma once

#include <cstdint>

namespace jb
{

// Starts the DWT cycle counter. Call once, before anything takes a timestamp.
void clock_init();

// Microseconds since clock_init(), 64-bit. Safe from any context: it masks interrupts for
// the few cycles it takes, because the IMU interrupt and the main loop both call it.
uint64_t now_us();

}  // namespace jb
