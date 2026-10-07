#include "bizon_behavior_servers/stockfish_process.hpp"

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>

#include <cerrno>
#include <cstring>
#include <exception>
#include <utility>

namespace bizon_behaviors
{

StockfishProcess::StockfishProcess(std::string binary)
: binary_(std::move(binary))
{
  // If execlp() in the child fails (binary not found), the child exits
  // immediately and closes its end of to_engine_. The next writeLine() call
  // in the parent then hits a pipe with no reader, which raises SIGPIPE --
  // whose default action is to kill the whole process, not just fail the
  // write. That turned "engine binary missing" into "behavior_server
  // crashes", silently, the first time anyone typo'd the binary name or
  // the engine died mid-game. Ignoring SIGPIPE once (it is a process-wide
  // disposition, so this only needs to happen somewhere before the first
  // write) turns it back into the ordinary EPIPE that write() already
  // handles like any other errno.
  ::signal(SIGPIPE, SIG_IGN);
}

StockfishProcess::~StockfishProcess()
{
  stop();
}

bool StockfishProcess::start(std::chrono::milliseconds timeout)
{
  if (isRunning()) {
    return true;
  }

  int to_pipe[2];
  int from_pipe[2];
  if (pipe(to_pipe) != 0 || pipe(from_pipe) != 0) {
    last_error_ = std::string("pipe() failed: ") + std::strerror(errno);
    return false;
  }

  const pid_t pid = fork();
  if (pid < 0) {
    last_error_ = std::string("fork() failed: ") + std::strerror(errno);
    return false;
  }

  if (pid == 0) {
    // Child: only async-signal-safe calls until execlp.
    ::close(to_pipe[1]);
    ::close(from_pipe[0]);
    ::dup2(to_pipe[0], STDIN_FILENO);
    ::dup2(from_pipe[1], STDOUT_FILENO);
    const int devnull = ::open("/dev/null", O_WRONLY);
    if (devnull >= 0) {
      ::dup2(devnull, STDERR_FILENO);
      ::close(devnull);
    }
    ::close(to_pipe[0]);
    ::close(from_pipe[1]);
    ::execlp(binary_.c_str(), binary_.c_str(), static_cast<char *>(nullptr));
    ::_exit(127);
  }

  ::close(to_pipe[0]);
  ::close(from_pipe[1]);
  to_engine_ = to_pipe[1];
  from_engine_ = from_pipe[0];
  pid_ = pid;
  pending_.clear();

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  std::string line;
  if (!writeLine("uci") || !readUntil("uciok", deadline, line)) {
    last_error_ = "engine did not answer uci handshake (" + last_error_ + ")";
    stop();
    return false;
  }
  if (!writeLine("isready") || !readUntil("readyok", deadline, line)) {
    last_error_ = "engine did not answer isready (" + last_error_ + ")";
    stop();
    return false;
  }

  last_error_.clear();
  return true;
}

void StockfishProcess::stop()
{
  if (to_engine_ >= 0) {
    writeLine("quit");
    ::close(to_engine_);
    to_engine_ = -1;
  }
  if (from_engine_ >= 0) {
    ::close(from_engine_);
    from_engine_ = -1;
  }
  if (pid_ > 0) {
    int status = 0;
    // Give the engine a moment to exit on `quit`, then insist.
    for (int i = 0; i < 20; ++i) {
      const pid_t r = ::waitpid(pid_, &status, WNOHANG);
      if (r == pid_ || r < 0) {
        pid_ = -1;
        return;
      }
      ::usleep(10000);
    }
    ::kill(pid_, SIGKILL);
    ::waitpid(pid_, &status, 0);
    pid_ = -1;
  }
}

bool StockfishProcess::setOption(const std::string & name, const std::string & value)
{
  return writeLine("setoption name " + name + " value " + value);
}

bool StockfishProcess::bestMove(
  const std::string & fen,
  int depth,
  std::chrono::milliseconds timeout,
  std::string & best_move_out)
{
  if (!isRunning()) {
    last_error_ = "engine is not running";
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;

  if (!writeLine("position fen " + fen) ||
    !writeLine("go depth " + std::to_string(depth)))
  {
    return false;
  }

  std::string line;
  if (!readUntil("bestmove", deadline, line)) {
    // Stop the search so the engine is reusable for the next goal.
    writeLine("stop");
    return false;
  }

  const size_t sp = line.find(' ');
  if (sp == std::string::npos || sp + 1 >= line.size()) {
    last_error_ = "malformed bestmove line: " + line;
    return false;
  }
  size_t end = line.find(' ', sp + 1);
  if (end == std::string::npos) {
    end = line.size();
  }
  best_move_out = line.substr(sp + 1, end - sp - 1);

  if (best_move_out.empty() || best_move_out == "(none)") {
    last_error_ = "engine reported no legal move";
    return false;
  }
  return true;
}

bool StockfishProcess::legalMoves(
  const std::string & fen,
  std::chrono::milliseconds timeout,
  std::vector<std::string> & moves_out)
{
  moves_out.clear();

  if (!isRunning()) {
    last_error_ = "engine is not running";
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;

  if (!writeLine("position fen " + fen) || !writeLine("go perft 1")) {
    return false;
  }

  std::string stop_line;
  std::vector<std::string> lines;
  if (!readUntil("Nodes searched:", deadline, stop_line, &lines)) {
    return false;
  }

  // Perft 1 prints one "<move>: 1" line per legal move, followed by the
  // "Nodes searched: N" summary line (captured as stop_line). N is
  // supposed to equal the number of moves printed, but that is an
  // assumption about the engine's output, not something the loop below
  // enforces on its own -- and DecisionPlugin::is_checkmate() treats an
  // empty moves_out as checkmate/stalemate. If a move line failed to parse
  // for any reason, an unchecked empty result would silently look
  // identical to a genuine mate and send the arm to sweep a king off the
  // board. So N is parsed out of stop_line below and required to match
  // what was actually parsed before this reports success.
  for (const auto & line : lines) {
    const size_t colon = line.find(':');
    if (colon == std::string::npos || line.rfind("Nodes searched:", 0) == 0) {
      continue;
    }
    const std::string move = line.substr(0, colon);
    if (move.size() == 4 || move.size() == 5) {
      moves_out.push_back(move);
    }
  }

  long reported_node_count = -1;
  const size_t summary_colon = stop_line.find(':');
  if (summary_colon != std::string::npos) {
    try {
      reported_node_count = std::stol(stop_line.substr(summary_colon + 1));
    } catch (const std::exception &) {
      reported_node_count = -1;
    }
  }

  if (reported_node_count < 0 || static_cast<size_t>(reported_node_count) != moves_out.size()) {
    last_error_ = "perft summary '" + stop_line + "' does not match the " +
      std::to_string(moves_out.size()) + " move line(s) parsed";
    moves_out.clear();
    return false;
  }

  return true;
}

bool StockfishProcess::applyMove(
  const std::string & base_fen,
  const std::string & move,
  std::chrono::milliseconds timeout,
  std::string & fen_out)
{
  if (!isRunning()) {
    last_error_ = "engine is not running";
    return false;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;

  if (!writeLine("position fen " + base_fen + " moves " + move) || !writeLine("d")) {
    return false;
  }

  std::string stop_line;
  std::vector<std::string> lines;
  if (!readUntil("Checkers:", deadline, stop_line, &lines)) {
    return false;
  }

  for (const auto & line : lines) {
    if (line.rfind("Fen: ", 0) == 0) {
      fen_out = line.substr(5);
      return true;
    }
  }

  last_error_ = "no 'Fen: ' line in 'd' output for move " + move;
  return false;
}

bool StockfishProcess::writeLine(const std::string & line)
{
  if (to_engine_ < 0) {
    last_error_ = "engine stdin is closed";
    return false;
  }
  const std::string payload = line + "\n";
  size_t written = 0;
  while (written < payload.size()) {
    const ssize_t n = ::write(to_engine_, payload.data() + written, payload.size() - written);
    if (n < 0) {
      if (errno == EINTR) {continue;}
      last_error_ = std::string("write() failed: ") + std::strerror(errno);
      return false;
    }
    written += static_cast<size_t>(n);
  }
  return true;
}

bool StockfishProcess::readUntil(
  const std::string & token,
  std::chrono::steady_clock::time_point deadline,
  std::string & line_out,
  std::vector<std::string> * captured_lines)
{
  char buffer[4096];

  while (true) {
    // Serve any complete line already buffered before touching the pipe.
    size_t nl = pending_.find('\n');
    while (nl != std::string::npos) {
      std::string line = pending_.substr(0, nl);
      pending_.erase(0, nl + 1);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      if (captured_lines != nullptr) {
        captured_lines->push_back(line);
      }
      if (line.rfind(token, 0) == 0) {
        line_out = line;
        return true;
      }
      nl = pending_.find('\n');
    }

    const auto now = std::chrono::steady_clock::now();
    if (now >= deadline) {
      last_error_ = "timed out waiting for '" + token + "'";
      return false;
    }

    const auto remaining =
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();

    struct pollfd pfd;
    pfd.fd = from_engine_;
    pfd.events = POLLIN;
    pfd.revents = 0;

    const int pr = ::poll(&pfd, 1, static_cast<int>(remaining));
    if (pr < 0) {
      if (errno == EINTR) {continue;}
      last_error_ = std::string("poll() failed: ") + std::strerror(errno);
      return false;
    }
    if (pr == 0) {
      last_error_ = "timed out waiting for '" + token + "'";
      return false;
    }

    const ssize_t n = ::read(from_engine_, buffer, sizeof(buffer));
    if (n < 0) {
      if (errno == EINTR) {continue;}
      last_error_ = std::string("read() failed: ") + std::strerror(errno);
      return false;
    }
    if (n == 0) {
      last_error_ = "engine closed its output pipe";
      return false;
    }
    pending_.append(buffer, static_cast<size_t>(n));
  }
}

}  // namespace bizon_behaviors
