#include "bizon_behavior_servers/debug_session.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <system_error>
#include <utility>

#include "bizon_behavior_servers/debug_session_format.hpp"

namespace bizon_behaviors
{
namespace fs = std::filesystem;

DebugSession & DebugSession::instance()
{
  static DebugSession session;
  return session;
}

void DebugSession::configure(const std::string & session_dir)
{
  if (session_dir.empty()) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (configured_) {
    return;  // first plugin to configure wins -- see the class comment
  }
  std::error_code ec;
  fs::create_directories(fs::path(session_dir) / "errors", ec);
  if (ec) {
    std::cerr << "[DebugSession] failed to create session directory '" << session_dir
              << "': " << ec.message() << std::endl;
    return;
  }
  session_dir_ = session_dir;
  configured_ = true;
}

bool DebugSession::isConfigured() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return configured_;
}

std::string DebugSession::sessionDir() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return session_dir_;
}

std::string DebugSession::movesFilePath() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!configured_) {
    return "";
  }
  return (fs::path(session_dir_) / "moves.txt").string();
}

namespace
{
// Shared by events.log and rosout.log: open-append-close per line rather
// than holding the file open for the process lifetime, so a crash never
// loses a buffered line and two processes (behavior_server and the rosout
// logger node, which each hold their own DebugSession instance -- they are
// different processes, not merely different plugin libraries) can append to
// their own respective files without coordinating a shared handle.
void appendLine(const std::string & path, const std::string & line)
{
  std::ofstream out(path, std::ios::app);
  if (!out.is_open()) {
    std::cerr << "[DebugSession] failed to open '" << path << "' for append" << std::endl;
    return;
  }
  out << line << '\n';
}
}  // namespace

void DebugSession::logEvent(const std::string & category, const std::string & body)
{
  std::string dir;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_) {
      return;
    }
    dir = session_dir_;
  }
  const std::string line =
    debug_format::formatLogLine(std::chrono::system_clock::now(), category, body);
  appendLine((fs::path(dir) / "events.log").string(), line);
}

void DebugSession::logRosout(const std::string & line)
{
  std::string dir;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_) {
      return;
    }
    dir = session_dir_;
  }
  appendLine((fs::path(dir) / "rosout.log").string(), line);
}

void DebugSession::registerImageDumpCallback(ImageDumpFn fn)
{
  std::lock_guard<std::mutex> lock(mutex_);
  image_dump_fn_ = std::move(fn);
}

bool DebugSession::isExpectedTransient(const std::string & label)
{
  // The camera cannot see the board while an arm is over it. That is the normal
  // state for most of every move, not a fault worth 5 MB of evidence.
  return label == "board_detect_failed";
}

std::string DebugSession::beginErrorBundle(const std::string & label)
{
  std::string dir;
  int counter = 0;
  int suppressed = 0;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!configured_) {
      return "";
    }

    const auto now = std::chrono::steady_clock::now();
    const auto it = last_bundle_at_.find(label);
    const bool cooling = it != last_bundle_at_.end() && (now - it->second) < kBundleCooldown;

    if (isExpectedTransient(label) || cooling) {
      const int n = ++suppressed_count_[label];
      // One line every 100 keeps the timeline honest about how often this is
      // firing without the log becoming the thing that fills the disk.
      if (n == 1 || n % 100 == 0) {
        dir = session_dir_;
      } else {
        return "";
      }
      suppressed = n;
    } else {
      dir = session_dir_;
      counter = ++error_counter_;
      last_bundle_at_[label] = now;
      suppressed = 0;
    }
  }

  if (counter == 0) {
    logEvent(
      "error", "label=" + label + " suppressed (no bundle) count=" + std::to_string(suppressed));
    return "";
  }

  const auto now = std::chrono::system_clock::now();
  const std::string bundle_name = debug_format::formatBundleDirName(counter, label, now);
  const fs::path bundle_dir = fs::path(dir) / "errors" / bundle_name;

  std::error_code ec;
  fs::create_directories(bundle_dir, ec);
  if (ec) {
    std::cerr << "[DebugSession] failed to create error bundle '" << bundle_dir.string()
              << "': " << ec.message() << std::endl;
    return "";
  }

  // Link the timeline to the evidence: a reader scanning events.log should
  // never have to guess which errors/ folder belongs to which failure.
  logEvent("error", "label=" + label + " bundle=errors/" + bundle_name);

  return bundle_dir.string();
}

void DebugSession::requestImageDump(const std::string & bundle_dir)
{
  if (bundle_dir.empty()) {
    return;
  }
  ImageDumpFn fn;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    fn = image_dump_fn_;
  }
  if (!fn) {
    return;
  }
  try {
    fn(bundle_dir);
  } catch (const std::exception & ex) {
    std::cerr << "[DebugSession] image dump callback threw for '" << bundle_dir
              << "': " << ex.what() << std::endl;
  } catch (...) {
    std::cerr << "[DebugSession] image dump callback threw a non-std::exception for '"
              << bundle_dir << "'" << std::endl;
  }
}

void DebugSession::writeContext(const std::string & bundle_dir, const std::string & text)
{
  if (bundle_dir.empty()) {
    return;
  }
  std::ofstream out((fs::path(bundle_dir) / "context.txt").string(), std::ios::app);
  if (!out.is_open()) {
    std::cerr << "[DebugSession] failed to open context.txt in '" << bundle_dir << "'"
              << std::endl;
    return;
  }
  out << text;
  if (!text.empty() && text.back() != '\n') {
    out << '\n';
  }
}

}  // namespace bizon_behaviors
