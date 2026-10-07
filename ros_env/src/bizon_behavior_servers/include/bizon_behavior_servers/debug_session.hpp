#ifndef BIZON_BEHAVIOR_SERVERS__DEBUG_SESSION_HPP_
#define BIZON_BEHAVIOR_SERVERS__DEBUG_SESSION_HPP_

#include <functional>
#include <chrono>
#include <map>
#include <mutex>
#include <string>

namespace bizon_behaviors
{

/// Process-wide game-session state, shared by every plugin loaded into one
/// behavior_server process. This is the seam the spec asks for: BoardPlugin
/// is the only plugin that ever touches an image, but DecisionPlugin is the
/// only plugin that knows a vision/AI failure happened, and they run as
/// separate pluginlib classes, each dlopen()'d from its own shared object
/// (see CMakeLists.txt: bizon_board_behavior and bizon_decision_behavior are
/// separate SHARED libraries). A Meyer's singleton compiled straight into
/// *each* plugin library would therefore NOT be one shared object at
/// runtime: dlopen() with the default (non-GLOBAL) flags does not merge
/// duplicate weak symbols across independently loaded .so files, so each
/// plugin would quietly get its own DebugSession with its own event counter
/// and its own idea of the session directory. Instead this class lives in
/// its own small shared library (bizon_debug_session) that every plugin
/// library links against with ordinary DT_NEEDED linkage; the dynamic
/// loader deduplicates *that* one by soname regardless of how pluginlib
/// loaded the plugins on top of it, so there really is exactly one instance
/// per behavior_server process -- which is the "process-wide session
/// object" the spec calls out as legitimate.
class DebugSession
{
public:
  static DebugSession & instance();

  /// Idempotent: the first plugin to configure wins and every later call is
  /// a no-op (matches all four plugins reading the same debug_session_dir
  /// parameter and each calling this from its own onConfigure()). Creates
  /// the session directory and its errors/ subdirectory if they do not
  /// exist yet. Passing an empty directory leaves the session unconfigured
  /// -- the normal state when a plugin runs outside the debug-logging
  /// launch wiring (e.g. under gtest), in which case every other method on
  /// this class is a harmless no-op.
  void configure(const std::string & session_dir);

  bool isConfigured() const;
  std::string sessionDir() const;

  /// "<session>/moves.txt". Empty when not configured.
  std::string movesFilePath() const;

  /// Appends one line to events.log: "<timestamp> | <category> | <body>".
  /// A no-op when the session is not configured; every call site treats
  /// this as fire-and-forget, matching "debug capture must never fail a
  /// goal".
  void logEvent(const std::string & category, const std::string & body);

  /// Appends one already-formatted line to rosout.log. Kept separate from
  /// logEvent(): the rosout logger node builds its own line (timestamp,
  /// level and logger name all come from the /rosout message, not this
  /// process's clock), but both funnel through the same append-only writer.
  void logRosout(const std::string & line);

  /// BoardPlugin registers this once, in its onConfigure(). It is the whole
  /// cross-plugin seam: DecisionPlugin has no images of its own, so when it
  /// detects a fen-mismatch or move-owner failure it calls
  /// requestImageDump() and BoardPlugin's callback writes whatever it
  /// currently knows into the given bundle directory.
  using ImageDumpFn = std::function<void (const std::string & bundle_dir)>;
  void registerImageDumpCallback(ImageDumpFn fn);

  /// Creates errors/<NNN>_<label>_<HH-MM-SS>/, logs the events.log line
  /// that links the timeline to it, and returns its path -- or "" if the
  /// bundle could not be created (session not configured, filesystem
  /// error), in which case the caller should skip the rest of the bundle
  /// and carry on. Only creates the directory; callers are expected to call
  /// writeContext() and then requestImageDump(), in that order, so that a
  /// partial write (disk fills up mid-bundle) leaves the highest-priority
  /// artifacts on disk -- see the priority order in requestImageDump()'s
  /// implementers (BoardPlugin::writeImageArtifacts).
  std::string beginErrorBundle(const std::string & label);

  /// Best-effort: invokes the registered image-dump callback, if any, with
  /// `bundle_dir`. Swallows and logs any exception the callback throws
  /// rather than letting it propagate into the caller's goal.
  void requestImageDump(const std::string & bundle_dir);

  /// Writes (or appends to) context.txt in the bundle directory.
  void writeContext(const std::string & bundle_dir, const std::string & text);

private:
  DebugSession() = default;

  /// Labels that describe an expected transient rather than a fault. The board
  /// is genuinely undetectable whenever an arm occludes it, which is most frames
  /// of every move -- bundling those produced 3623 directories and 18 GB in one
  /// game. They are counted in events.log and never given a directory.
  static bool isExpectedTransient(const std::string & label);

  /// A bundle costs ~5 MB of images, so the same label is only bundled once per
  /// cooldown. Repeats in between are counted and reported, not written.
  static constexpr std::chrono::seconds kBundleCooldown{30};

  mutable std::mutex mutex_;
  std::map<std::string, std::chrono::steady_clock::time_point> last_bundle_at_;
  std::map<std::string, int> suppressed_count_;
  std::string session_dir_;
  bool configured_{false};
  int error_counter_{0};
  ImageDumpFn image_dump_fn_;
};

}  // namespace bizon_behaviors

#endif  // BIZON_BEHAVIOR_SERVERS__DEBUG_SESSION_HPP_
