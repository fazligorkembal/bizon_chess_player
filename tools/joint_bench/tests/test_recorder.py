import os

import pytest
import yaml

from fake_port import FakeClock, FakeFirmware
from joint_bench.recorder import Link, RecordError, record


def _record(tmp_path, fw, clock, seconds=1, marks=()):
    out = tmp_path / "rec"
    meta = record(Link(fw, clock=clock), str(out), seconds, "test", str(tmp_path), marks=marks)
    return out, meta


def test_records_the_requested_duration_of_firmware_time(tmp_path):
    clock = FakeClock()
    fw = FakeFirmware(clock)
    out, meta = _record(tmp_path, fw, clock, seconds=1)
    rows = (out / "raw.csv").read_text().splitlines()
    assert rows[0] == "seq,t_us,ax,ay,az,gx,gy,gz"
    assert len(rows) - 1 == 1001  # t = 0 .. 1 000 000 us inclusive
    assert meta["imu"]["lsb_per_g"] == "8192"
    assert meta["stats"]["i2c_err"] == "0"
    assert fw.sent[-2:] == ["stream off", "stats"]
    assert yaml.safe_load((out / "meta.yaml").read_text())["label"] == "test"


def test_sends_scheduled_marks_and_records_their_events(tmp_path):
    clock = FakeClock()
    fw = FakeFirmware(clock)
    out, meta = _record(tmp_path, fw, clock, seconds=1, marks=[(0.2, "a7-1"), (0.5, "a7-2")])
    assert "mark a7-1" in fw.sent and "mark a7-2" in fw.sent
    events = (out / "events.csv").read_text().splitlines()
    assert events[0] == "t_us,name"
    assert [e.split(",")[1] for e in events[1:]] == ["mark:a7-1", "mark:a7-2"]
    assert meta["events"] == 2


def test_a_dead_link_raises_and_leaves_no_meta(tmp_path):
    clock = FakeClock()
    fw = FakeFirmware(clock, stall_after=100)
    with pytest.raises(RecordError, match="stalled"):
        _record(tmp_path, fw, clock, seconds=1)
    assert not os.path.exists(tmp_path / "rec" / "meta.yaml")


def test_refuses_to_overwrite_an_existing_recording(tmp_path):
    (tmp_path / "rec").mkdir()
    clock = FakeClock()
    with pytest.raises(FileExistsError):
        _record(tmp_path, FakeFirmware(clock), clock)
