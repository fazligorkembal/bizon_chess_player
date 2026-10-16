#include "jb/telemetry.hpp"

#include "jb/text_writer.hpp"

namespace jb
{

uint32_t format_sample(char * buf, uint32_t cap, const ImuSample & s)
{
  TextWriter w(buf, cap);
  w.str("T,");
  w.u64(s.seq);
  w.put(',');
  w.u64(s.t_us);
  for (int16_t v : s.accel) {
    w.put(',');
    w.i32(v);
  }
  for (int16_t v : s.gyro) {
    w.put(',');
    w.i32(v);
  }
  w.put('\n');
  return w.ok() ? w.size() : 0;
}

uint32_t format_event(char * buf, uint32_t cap, uint64_t t_us, std::string_view name)
{
  TextWriter w(buf, cap);
  w.str("E,");
  w.u64(t_us);
  w.put(',');
  w.str(name);
  w.put('\n');
  return w.ok() ? w.size() : 0;
}

}  // namespace jb
