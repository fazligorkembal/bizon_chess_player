import os

import pytest
import yaml

from fake_port import FakeClock, FakeFirmware
from joint_bench.recorder import Link, RecordError, record


def _record(tmp_path, fw, clock, seconds=1, marks=()):
    out = tmp_path / "rec"
    meta = record(Link(fw, clock=clock), str(out), seconds, "test", str(tmp_path), marks=marks)
    return out, meta


def _answering_imu_info_with(fw, reply):
    write = fw.write

    def patched(data):
        write(data)
        if data.decode().strip() == "imu info":
            fw.out.append(reply)
    return patched


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


def test_a_non_ascii_line_mid_stream_is_logged_escaped_and_the_recording_completes(tmp_path):
    clock = FakeClock()
    fw = FakeFirmware(clock, noise_at=50)
    out, meta = _record(tmp_path, fw, clock, seconds=1)
    assert meta["unknown_lines"] >= 1
    assert "\\xff\\xfe" in (out / "log.txt").read_text(encoding="ascii")
    assert (out / "meta.yaml").exists()


def test_a_lost_reply_while_streaming_times_out_instead_of_spinning():
    clock = FakeClock()
    fw = FakeFirmware(clock, mute=("stream off",))
    fw.streaming = True
    with pytest.raises(RecordError, match="no reply"):
        Link(fw, clock=clock).command("stream off", timeout_s=0.5)


def test_stale_replies_do_not_pair_with_the_handshake(tmp_path):
    clock = FakeClock()
    fw = FakeFirmware(clock)
    fw.streaming = True
    fw.out.extend(["ERR unknown command\n"] * 16)
    _, meta = _record(tmp_path, fw, clock)
    assert meta["firmware"] == "ver joint_bench test"
    assert meta["imu"]["lsb_per_g"] == "8192"


def test_a_stream_off_reply_lost_to_a_full_ring_is_asked_for_again(tmp_path):
    clock = FakeClock()
    fw = FakeFirmware(clock)
    fw.streaming = True
    write = fw.write
    lost = []

    def lose_first_reply(data):
        write(data)
        if not lost:
            lost.append(fw.out.pop())

    fw.write = lose_first_reply
    _, meta = _record(tmp_path, fw, clock)
    assert lost == ["OK stream off\n"]
    assert meta["imu"]["lsb_per_g"] == "8192"


def test_an_imu_info_reply_without_a_scale_stops_the_recording(tmp_path):
    clock = FakeClock()
    fw = FakeFirmware(clock, mute=("imu info",))
    fw.write = _answering_imu_info_with(fw, "OK imu chip=MPU6050\n")
    with pytest.raises(RecordError, match="unexpected reply"):
        _record(tmp_path, fw, clock)


def test_record_raises_when_stream_off_is_never_answered(tmp_path):
    clock = FakeClock()
    fw = FakeFirmware(clock, mute=("stream off",))
    with pytest.raises(RecordError, match="no reply"):
        _record(tmp_path, fw, clock, seconds=1)
    assert not os.path.exists(tmp_path / "rec" / "meta.yaml")
