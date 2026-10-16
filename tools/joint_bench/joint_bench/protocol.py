"""Parse the firmware's line protocol.

Every line the firmware sends is one of: a sample (`T,...`), an event (`E,...`), a command
reply (`OK ...` / `ERR ...`) or a log line (`# ...`). Anything else is line noise and is
returned as `Unknown` so the caller can count it rather than crash on it.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class Sample:
    seq: int
    t_us: int
    accel: tuple  # raw counts, x y z
    gyro: tuple   # raw counts, x y z


@dataclass(frozen=True)
class Event:
    t_us: int
    name: str


@dataclass(frozen=True)
class Reply:
    ok: bool
    text: str  # everything after "OK " / "ERR "

    def fields(self):
        """key=value pairs in the reply, e.g. {'chip': 'MPU6050', 'rate': '1000'}."""
        out = {}
        for word in self.text.split():
            key, sep, value = word.partition("=")
            if sep:
                out[key] = value
        return out


@dataclass(frozen=True)
class Log:
    text: str


@dataclass(frozen=True)
class Unknown:
    text: str


def parse_line(line):
    line = line.rstrip("\r\n")
    if line.startswith("T,"):
        parts = line.split(",")
        if len(parts) == 9:
            try:
                nums = [int(p) for p in parts[1:]]
            except ValueError:
                return Unknown(line)
            return Sample(nums[0], nums[1], tuple(nums[2:5]), tuple(nums[5:8]))
        return Unknown(line)
    if line.startswith("E,"):
        parts = line.split(",", 2)
        if len(parts) == 3 and parts[1].isdigit():
            return Event(int(parts[1]), parts[2])
        return Unknown(line)
    if line == "OK" or line.startswith("OK "):
        return Reply(True, line[3:])
    if line == "ERR" or line.startswith("ERR "):
        return Reply(False, line[4:])
    if line.startswith("#"):
        return Log(line[1:].strip())
    return Unknown(line)
