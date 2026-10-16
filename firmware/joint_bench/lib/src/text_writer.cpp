#include "jb/text_writer.hpp"

namespace jb
{

void TextWriter::put(char c)
{
  if (!ok_ || len_ == cap_) {
    ok_ = false;
    return;
  }
  buf_[len_++] = c;
}

void TextWriter::str(std::string_view text)
{
  for (char c : text) {
    put(c);
  }
}

void TextWriter::u64(uint64_t value)
{
  char digits[20];
  int n = 0;
  do {
    digits[n++] = static_cast<char>('0' + value % 10);
    value /= 10;
  } while (value != 0);
  while (n > 0) {
    put(digits[--n]);
  }
}

void TextWriter::i32(int32_t value)
{
  // Negate in 64 bits: -INT32_MIN does not fit in an int32_t.
  int64_t wide = value;
  if (wide < 0) {
    put('-');
    wide = -wide;
  }
  u64(static_cast<uint64_t>(wide));
}

void TextWriter::hex8(uint8_t value)
{
  static constexpr char kHex[] = "0123456789abcdef";
  put(kHex[value >> 4]);
  put(kHex[value & 0x0f]);
}

}  // namespace jb
