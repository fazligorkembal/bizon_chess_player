#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "jb/line_assembler.hpp"

namespace
{
// Feeds text and returns what came out: lines as text, overflows as "<overflow>".
std::vector<std::string> feed(jb::LineAssembler & la, const std::string & text)
{
  std::vector<std::string> out;
  for (char c : text) {
    switch (la.feed(c)) {
      case jb::LineAssembler::Result::Line: out.emplace_back(la.line()); break;
      case jb::LineAssembler::Result::Overflow: out.emplace_back("<overflow>"); break;
      case jb::LineAssembler::Result::None: break;
    }
  }
  return out;
}
}  // namespace

TEST(LineAssembler, AcceptsLfCrAndCrlfAndSkipsEmptyLines)
{
  jb::LineAssembler la;
  EXPECT_EQ(feed(la, "ver\nstats\r\n\r\ni2cscan\r"),
    (std::vector<std::string>{"ver", "stats", "i2cscan"}));
}

TEST(LineAssembler, DiscardsAnOverlongLineWholeAndRecovers)
{
  jb::LineAssembler la;
  const std::string longline(jb::LineAssembler::kMax + 10, 'a');
  EXPECT_EQ(feed(la, longline + "\nver\n"), (std::vector<std::string>{"<overflow>", "ver"}));
}

TEST(LineAssembler, AcceptsALineOfExactlyMaxLength)
{
  jb::LineAssembler la;
  const std::string line(jb::LineAssembler::kMax, 'b');
  EXPECT_EQ(feed(la, line + "\n"), (std::vector<std::string>{line}));
}
