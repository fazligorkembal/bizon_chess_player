#pragma once

#include <cstdint>
#include <string_view>

namespace jb
{

// Appends text and integers to a fixed buffer. newlib-nano's printf, which the firmware
// links, has no %llu, and the timestamps are 64-bit, so numbers are formatted by hand.
// Writing never passes `cap`; once anything fails to fit, ok() stays false and the caller
// discards the line instead of sending a truncated one.
class TextWriter
{
public:
  TextWriter(char * buf, uint32_t cap)
  : buf_(buf), cap_(cap) {}

  void put(char c);
  void str(std::string_view text);
  void u64(uint64_t value);
  void i32(int32_t value);
  void hex8(uint8_t value);  // always two digits, lower case, no prefix

  bool ok() const {return ok_;}
  uint32_t size() const {return len_;}

private:
  char * buf_;
  uint32_t cap_;
  uint32_t len_ = 0;
  bool ok_ = true;
};

}  // namespace jb
