"""A scripted stand-in for the firmware's serial port."""

from collections import deque


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


class FakeFirmware:
    """Answers commands like the firmware does and streams samples at 1 kHz while on.

    Each readline() advances the fake clock by one millisecond. `drop` lists seq numbers
    the stream skips, to imitate lost samples. `stall_after` stops the stream after that
    many samples, to imitate a dead link.
    """

    def __init__(self, clock, drop=(), stall_after=None):
        self.clock = clock
        self.out = deque()
        self.streaming = False
        self.seq = 0
        self.drop = set(drop)
        self.stall_after = stall_after
        self.sent = []

    def write(self, data):
        cmd = data.decode().strip()
        self.sent.append(cmd)
        if cmd == "stream on":
            self.streaming = True
            self.out.append("OK stream on\n")
        elif cmd == "stream off":
            self.streaming = False
            self.out.append("OK stream off\n")
        elif cmd == "ver":
            self.out.append("OK ver joint_bench test\n")
        elif cmd == "imu info":
            self.out.append("OK imu chip=MPU6050 who=0x68 addr=0x68 rate=1000 dlpf=184 fs=4"
                            " lsb_per_g=8192\n")
        elif cmd == "stats":
            self.out.append(f"OK stats samples={self.seq} imu_overrun=0 i2c_err=0 tx_drop=0"
                            " queue_drop=0\n")
        elif cmd.startswith("mark"):
            label = cmd.partition(" ")[2]
            name = f"mark:{label}" if label else "mark"
            self.out.append(f"E,{self.seq * 1000 + 300},{name}\n")
            self.out.append("OK mark\n")
        else:
            self.out.append("ERR unknown command\n")

    def readline(self):
        self.clock.t += 0.001
        if self.out:
            return self.out.popleft().encode()
        if self.streaming and (self.stall_after is None or self.seq < self.stall_after):
            seq = self.seq
            self.seq += 1
            if seq in self.drop:
                return self.readline()
            return f"T,{seq},{seq * 1000},0,0,8192,0,0,0\n".encode()
        return b""
