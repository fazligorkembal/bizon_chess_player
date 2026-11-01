#include "imu.hpp"

#include "clock.hpp"
#include "i2c.h"
#include "jb/spsc_queue.hpp"
#include "main.h"

namespace jb
{
namespace
{
constexpr uint32_t kTimeoutMs = 10;

mpu::Chip g_chip = mpu::Chip::Unknown;
uint8_t g_who = 0;
uint8_t g_addr = 0;
mpu::Plan g_plan{};

volatile bool g_streaming = false;
volatile bool g_busy = false;
uint32_t g_seq = 0;
uint64_t g_t_us = 0;           // timestamp of the edge whose read is in flight
uint32_t g_read_seq = 0;
uint8_t g_block[mpu::kBlockLen];
SpscQueue<ImuSample, 64> g_queue;
volatile ImuCounters g_count{};

// Blocking bus use must wait out a DMA read that was already in flight when streaming
// stopped; starting a polled transfer underneath it corrupts both.
bool wait_idle()
{
  const uint32_t start = HAL_GetTick();
  while (g_busy) {
    if (HAL_GetTick() - start > kTimeoutMs) {
      return false;
    }
  }
  return true;
}

HAL_StatusTypeDef write_reg(uint8_t reg, uint8_t value)
{
  return HAL_I2C_Mem_Write(&hi2c1, static_cast<uint16_t>(g_addr << 1), reg, I2C_MEMADD_SIZE_8BIT,
                           &value, 1, kTimeoutMs);
}

const char * apply(const mpu::Config & cfg)
{
  mpu::Plan plan{};
  if (const char * err = mpu::plan_config(g_chip, cfg, &plan)) {
    return err;
  }
  if (write_reg(mpu::reg::kPwrMgmt1, mpu::kResetValue) != HAL_OK) {
    return "imu: reset write failed";
  }
  HAL_Delay(100);
  for (uint8_t i = 0; i < plan.count; ++i) {
    if (write_reg(plan.writes[i].reg, plan.writes[i].value) != HAL_OK) {
      return "imu: config write failed";
    }
  }
  g_plan = plan;
  return nullptr;
}
}  // namespace

const char * imu_probe(const mpu::Config & cfg)
{
  if (!wait_idle()) {
    return "imu: bus busy";
  }
  g_chip = mpu::Chip::Unknown;
  for (uint8_t addr : {mpu::kAddrLow, mpu::kAddrHigh}) {
    uint8_t who = 0;
    if (HAL_I2C_Mem_Read(&hi2c1, static_cast<uint16_t>(addr << 1), mpu::reg::kWhoAmI,
      I2C_MEMADD_SIZE_8BIT, &who, 1, kTimeoutMs) != HAL_OK)
    {
      continue;
    }
    g_addr = addr;
    g_who = who;
    g_chip = mpu::identify(who);
    if (g_chip == mpu::Chip::Unknown) {
      return "imu: unsupported who_am_i";
    }
    return apply(cfg);
  }
  return "imu: not found";
}

const char * imu_configure(const mpu::Config & cfg)
{
  if (g_chip == mpu::Chip::Unknown) {
    return "imu: not found";
  }
  if (!wait_idle()) {
    return "imu: bus busy";
  }
  return apply(cfg);
}

bool imu_present() {return g_chip != mpu::Chip::Unknown;}
mpu::Chip imu_chip() {return g_chip;}
uint8_t imu_who_am_i() {return g_who;}
uint8_t imu_address() {return g_addr;}
const mpu::Plan & imu_plan() {return g_plan;}

void imu_stream(bool on)
{
  g_streaming = on;
}

bool imu_streaming() {return g_streaming;}

bool imu_pop(ImuSample * out)
{
  return g_queue.pop(out);
}

ImuCounters imu_counters()
{
  __disable_irq();
  const ImuCounters c{g_count.samples, g_count.overrun, g_count.i2c_err, g_count.queue_drop};
  __enable_irq();
  return c;
}

int imu_scan(uint8_t * found, int max)
{
  if (!wait_idle()) {
    return 0;
  }
  int n = 0;
  for (uint8_t addr = 0x08; addr <= 0x77 && n < max; ++addr) {
    if (HAL_I2C_IsDeviceReady(&hi2c1, static_cast<uint16_t>(addr << 1), 2, 2) == HAL_OK) {
      found[n++] = addr;
    }
  }
  return n;
}

// Every data-ready edge consumes a sequence number, whether or not its sample survives.
// That is what turns every loss downstream (overrun, bus error, full queue, full USB
// ring) into a gap the host can count, instead of a silent shortening of the recording.
void imu_on_data_ready()
{
  if (!g_streaming) {
    return;
  }
  const uint32_t seq = g_seq++;
  g_count.samples = g_count.samples + 1;
  if (g_busy) {
    g_count.overrun = g_count.overrun + 1;
    return;
  }
  g_t_us = now_us();
  g_read_seq = seq;
  g_busy = true;
  if (HAL_I2C_Mem_Read_DMA(&hi2c1, static_cast<uint16_t>(g_addr << 1), mpu::reg::kAccelXoutH,
    I2C_MEMADD_SIZE_8BIT, g_block, mpu::kBlockLen) != HAL_OK)
  {
    g_busy = false;
    g_count.i2c_err = g_count.i2c_err + 1;
  }
}

void imu_on_read_done()
{
  const mpu::Raw raw = mpu::parse_block(g_block);
  const ImuSample s{g_read_seq, g_t_us, {raw.accel[0], raw.accel[1], raw.accel[2]},
    {raw.gyro[0], raw.gyro[1], raw.gyro[2]}};
  if (!g_queue.push(s)) {
    g_count.queue_drop = g_count.queue_drop + 1;
  }
  g_busy = false;
}

void imu_on_read_error()
{
  g_count.i2c_err = g_count.i2c_err + 1;
  g_busy = false;
}

}  // namespace jb

extern "C" void HAL_GPIO_EXTI_Callback(uint16_t pin)
{
  if (pin == IMU_INT_Pin) {
    jb::imu_on_data_ready();
  }
}

extern "C" void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef * hi2c)
{
  if (hi2c == &hi2c1) {
    jb::imu_on_read_done();
  }
}

extern "C" void HAL_I2C_ErrorCallback(I2C_HandleTypeDef * hi2c)
{
  if (hi2c == &hi2c1) {
    jb::imu_on_read_error();
  }
}
