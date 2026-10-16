"""Command line for the joint0 bench.

    python -m joint_bench console                 talk to the firmware by hand
    python -m joint_bench record -s 60 -l rest    record 60 s into debug/bench/
    python -m joint_bench analyze <dir>           analyse a recording

Run from tools/joint_bench, or with it on PYTHONPATH.
"""

import argparse
import datetime
import os
import sys
import threading

from . import recorder, report
from .protocol import Sample, parse_line

# Four levels up, not three: __main__.py sits in tools/joint_bench/joint_bench/, and the
# recordings belong in the repository root's debug/bench, not under tools/.
REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
DEFAULT_OUT = os.path.join(REPO, "debug", "bench")
ST_VCP = (0x0483, 0x5740)  # STMicroelectronics Virtual COM Port, CubeMX's CDC default


def find_port():
    from serial.tools import list_ports
    hits = [p.device for p in list_ports.comports() if (p.vid, p.pid) == ST_VCP]
    if len(hits) != 1:
        raise SystemExit(f"expected one ST virtual COM port, found {hits or 'none'}; pass --port")
    return hits[0]


def open_port(name):
    import serial
    return serial.Serial(name or find_port(), baudrate=115200, timeout=0.1)


def cmd_console(args):
    port = open_port(args.port)
    link = recorder.Link(port)
    stop = threading.Event()

    def reader():
        samples = 0
        for line in link.lines():
            if stop.is_set():
                return
            if line is None:
                continue
            if isinstance(parse_line(line), Sample):
                samples += 1
                if samples % 1000 == 0:
                    print(f"  ... {samples} samples", flush=True)
                continue
            print(line.rstrip(), flush=True)

    threading.Thread(target=reader, daemon=True).start()
    print("type commands, 'help' lists them, Ctrl-D quits")
    try:
        for line in sys.stdin:
            link.send(line.strip())
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        link.send("stream off")


def cmd_record(args):
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir = os.path.join(args.out, f"{stamp}_{args.label}")
    marks = [(args.mark_start + i * args.mark_every, f"{args.label}-{i + 1}")
             for i in range(args.marks)]
    port = open_port(args.port)
    try:
        meta = recorder.record(recorder.Link(port), outdir, args.seconds, args.label, REPO,
                               marks=marks)
    finally:
        port.close()
    print(f"{meta['samples']} samples, {meta['events']} events -> {outdir}")
    print(report.summary(report.analyze(outdir)))


def cmd_analyze(args):
    print(report.summary(report.analyze(args.dir, noise_from=args.noise_from)))


def main(argv=None):
    parser = argparse.ArgumentParser(prog="joint_bench")
    sub = parser.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("console")
    p.add_argument("--port")
    p.set_defaults(func=cmd_console)
    p = sub.add_parser("record")
    p.add_argument("-s", "--seconds", type=float, required=True)
    p.add_argument("-l", "--label", required=True)
    p.add_argument("--port")
    p.add_argument("--out", default=DEFAULT_OUT)
    p.add_argument("--marks", type=int, default=0,
                   help="send this many marks during the recording")
    p.add_argument("--mark-start", type=float, default=1.0)
    p.add_argument("--mark-every", type=float, default=1.0)
    p.set_defaults(func=cmd_record)
    p = sub.add_parser("analyze")
    p.add_argument("dir")
    p.add_argument("--noise-from", help="a rest recording to take the noise floor from")
    p.set_defaults(func=cmd_analyze)
    args = parser.parse_args(argv)
    args.func(args)


if __name__ == "__main__":
    main()
