#pragma once

#include <cstdint>

#include "jb/mpu.hpp"
#include "jb/telemetry.hpp"

namespace jb
{

struct ImuCounters
{
  uint32_t samples;    // data-ready edges seen while streaming
  uint32_t overrun;    // an edge arrived while the previous read was still in flight
  uint32_t i2c_err;    // a read could not start or ended in a bus error
  uint32_t queue_drop; // the main loop fell behind and the sample queue was full
};

// Probes 0x68 then 0x69, identifies the chip and applies `cfg` after a reset. Blocking;
// never call it while streaming. Returns nullptr on success or an error message.
const char * imu_probe(const mpu::Config & cfg);

// Reconfigures a chip already found. Blocking; never call it while streaming.
const char * imu_configure(const mpu::Config & cfg);

bool imu_present();
mpu::Chip imu_chip();
uint8_t imu_who_am_i();
uint8_t imu_address();
const mpu::Plan & imu_plan();

void imu_stream(bool on);
bool imu_streaming();

// Main loop: the next sample the interrupt path produced, if any.
bool imu_pop(ImuSample * out);

ImuCounters imu_counters();

// Blocking scan of 0x08..0x77. Writes up to `max` addresses, returns how many were found.
int imu_scan(uint8_t * found, int max);

// Interrupt-side hooks, called from the HAL callbacks in imu.cpp.
void imu_on_data_ready();
void imu_on_read_done();
void imu_on_read_error();

}  // namespace jb
