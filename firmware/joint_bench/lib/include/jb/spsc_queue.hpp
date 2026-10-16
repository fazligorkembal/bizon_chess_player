#pragma once

#include <atomic>
#include <cstdint>

namespace jb
{

// Lock-free queue for exactly one producer and one consumer. The IMU's DMA-complete
// interrupt pushes samples and the main loop pops them, so the interrupt never waits on
// USB and the main loop never runs with interrupts masked. N must be a power of two so
// the free-running indices stay correct across their own wrap.
template<typename T, uint32_t N>
class SpscQueue
{
  static_assert(N != 0 && (N & (N - 1)) == 0, "N must be a power of two");

public:
  // Producer side. Returns false, and stores nothing, when the queue is full.
  bool push(const T & value)
  {
    const uint32_t head = head_.load(std::memory_order_relaxed);
    const uint32_t tail = tail_.load(std::memory_order_acquire);
    if (head - tail == N) {
      return false;
    }
    buf_[head & (N - 1)] = value;
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  // Consumer side. Returns false when the queue is empty.
  bool pop(T * out)
  {
    const uint32_t tail = tail_.load(std::memory_order_relaxed);
    const uint32_t head = head_.load(std::memory_order_acquire);
    if (head == tail) {
      return false;
    }
    *out = buf_[tail & (N - 1)];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

  uint32_t size() const
  {
    return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
  }

private:
  T buf_[N] {};
  std::atomic<uint32_t> head_{0};
  std::atomic<uint32_t> tail_{0};
};

}  // namespace jb
