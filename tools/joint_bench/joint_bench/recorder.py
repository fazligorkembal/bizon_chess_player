"""Record one IMU stream to disk.

The recording directory holds raw.csv (samples), events.csv (marks), log.txt (everything
else the firmware said) and meta.yaml (what produced the data). meta.yaml is written last,
so a directory without one is an interrupted recording and the analyser refuses it.
"""

import datetime
import os
import subprocess
import time

import yaml

from .protocol import Event, Reply, Sample, Unknown, parse_line

STALL_S = 2.0  # no line for this long while streaming means the link is gone


class RecordError(RuntimeError):
    pass


def git_sha(repo):
    try:
        out = subprocess.run(["git", "-C", repo, "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, check=True)
        return out.stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


class Link:
    """Line-level wrapper around a pyserial-like port: write(bytes), readline() -> bytes."""

    def __init__(self, port, clock=time.monotonic):
        self.port = port
        self.clock = clock

    def send(self, command):
        self.port.write((command + "\n").encode("ascii"))

    def lines(self):
        while True:
            raw = self.port.readline()
            if not raw:
                yield None  # read timeout
                continue
            # backslashreplace, not replace: line noise must reach the log as ASCII, since
            # the recording files are ASCII and a U+FFFD would abort the whole recording.
            yield raw.decode("ascii", errors="backslashreplace")

    def command(self, command, timeout_s=2.0, on_other=None):
        """Send a command and return its Reply. Lines that arrive first go to on_other."""
        self.send(command)
        deadline = self.clock() + timeout_s
        for line in self.lines():
            # The deadline is checked on every line, not only on read timeouts: while the
            # stream flows at 1 kHz there is never a timeout, so a lost reply would spin.
            if line is not None:
                msg = parse_line(line)
                if isinstance(msg, Reply):
                    return msg
                if on_other:
                    on_other(msg)
            if self.clock() > deadline:
                raise RecordError(f"no reply to {command!r}")


def record(link, outdir, seconds, label, repo, marks=()):
    """Stream for `seconds` of firmware time into outdir. Returns the meta dict.

    `marks` is a list of (seconds after the stream starts, label): the recorder sends
    `mark <label>` at each, since it holds the port and nothing else can.
    """
    os.makedirs(outdir, exist_ok=False)
    others = []
    link.command("stream off", on_other=others.append)
    ver = link.command("ver")
    info = link.command("imu info")
    if not info.ok:
        raise RecordError(f"imu info failed: {info.text}")
    fields = info.fields()
    samples = events = unknown = 0
    first_t = last_t = None
    with open(os.path.join(outdir, "raw.csv"), "w", encoding="ascii") as raw, \
            open(os.path.join(outdir, "events.csv"), "w", encoding="ascii") as ev, \
            open(os.path.join(outdir, "log.txt"), "w", encoding="ascii") as log:
        raw.write("seq,t_us,ax,ay,az,gx,gy,gz\n")
        ev.write("t_us,name\n")
        reply = link.command("stream on")
        if not reply.ok:
            raise RecordError(f"stream on failed: {reply.text}")
        start = last_line = link.clock()
        pending = sorted(marks)
        for line in link.lines():
            if pending and link.clock() - start >= pending[0][0]:
                link.send(f"mark {pending.pop(0)[1]}")
            now = link.clock()
            if line is None:
                if now - last_line > STALL_S:
                    raise RecordError(f"stream stalled after {samples} samples")
                continue
            last_line = now
            msg = parse_line(line)
            if isinstance(msg, Sample):
                raw.write(f"{msg.seq},{msg.t_us},{','.join(map(str, msg.accel))},"
                          f"{','.join(map(str, msg.gyro))}\n")
                samples += 1
                first_t = msg.t_us if first_t is None else first_t
                last_t = msg.t_us
                if last_t - first_t >= seconds * 1_000_000:
                    break
            elif isinstance(msg, Event):
                ev.write(f"{msg.t_us},{msg.name}\n")
                events += 1
            elif isinstance(msg, Unknown):
                unknown += 1
                log.write(f"? {msg.text}\n")
            else:
                log.write(f"{msg}\n")
        link.command("stream off", on_other=lambda m: None)
        stats = link.command("stats")
    meta = {
        "label": label,
        "started": datetime.datetime.now().astimezone().isoformat(timespec="seconds"),
        "seconds": seconds,
        "samples": samples,
        "events": events,
        "unknown_lines": unknown,
        "firmware": ver.text,
        "imu": fields,
        # A failed stats reply is recorded as such, so the report can fail A4 on it instead
        # of reading an empty dict as "no errors".
        "stats": stats.fields() if stats.ok else {"error": stats.text},
        "git": git_sha(repo),
    }
    with open(os.path.join(outdir, "meta.yaml"), "w", encoding="utf-8") as fh:
        yaml.safe_dump(meta, fh, sort_keys=False)
    return meta
