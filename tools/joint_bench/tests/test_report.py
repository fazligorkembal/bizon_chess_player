import os

import numpy as np
import pytest
import yaml

from joint_bench import analysis, report


def _write_recording(path, a_g, lsb=8192, rate=1000, drop=(), stats={"i2c_err": "0"},
                     gyro_counts=None):
    os.makedirs(path)
    n = len(a_g)
    seq = np.array([i for i in range(n + len(drop)) if i not in set(drop)])[:n]
    counts = np.round(a_g * lsb).astype(int)
    if gyro_counts is None:
        gyro_counts = np.zeros((n, 3), dtype=int)
    with open(os.path.join(path, "raw.csv"), "w") as fh:
        fh.write("seq,t_us,ax,ay,az,gx,gy,gz\n")
        for s, c, w in zip(seq, counts, gyro_counts):
            fh.write(f"{s},{s * 1000},{c[0]},{c[1]},{c[2]},{w[0]},{w[1]},{w[2]}\n")
    with open(os.path.join(path, "events.csv"), "w") as fh:
        fh.write("t_us,name\n2300,mark:a7-1\n")
    meta = {"label": "synthetic", "imu": {"rate": str(rate), "lsb_per_g": str(lsb)}}
    if stats is not None:
        meta["stats"] = stats
    with open(os.path.join(path, "meta.yaml"), "w") as fh:
        yaml.safe_dump(meta, fh)


def test_a_clean_tapped_recording_passes_and_reports_the_tap(tmp_path):
    rng = np.random.default_rng(3)
    n = 6000
    t = np.arange(n) / 1000
    a = rng.normal(0, 0.001, size=(n, 3))
    a[:, 2] += 1.0
    tt = t - 2.0
    ring = 0.2 * np.exp(-0.04 * 2 * np.pi * 45 * tt) * np.sin(2 * np.pi * 45 * tt)
    a[:, 0] += np.where(tt >= 0, ring, 0)
    path = str(tmp_path / "rec")
    _write_recording(path, a)
    result = report.analyze(path)
    assert all(result["checks"].values()), result["checks"]
    assert len(result["taps"]) == 1
    assert abs(result["taps"][0]["freq_hz"] - 45) < 1
    assert result["marks"] == [{"name": "mark:a7-1", "to_next_sample_us": 700}]
    for name in ("analysis.json", "horizontal.png", "intervals.png", "psd.png"):
        assert os.path.exists(os.path.join(path, name))
    assert "PASS  A3 no missing samples" in report.summary(result)


def test_missing_samples_fail_a3(tmp_path):
    a = np.tile([0.0, 0.0, 1.0], (3000, 1))
    path = str(tmp_path / "rec")
    _write_recording(path, a, drop=(100, 101))
    result = report.analyze(path)
    assert result["checks"]["A3 no missing samples"] is False


@pytest.mark.parametrize("stats", [None, {}, {"error": "ERR busy"}])
def test_a4_fails_without_a_clean_i2c_error_count(tmp_path, stats):
    a = np.tile([0.0, 0.0, 1.0], (3000, 1))
    path = str(tmp_path / "rec")
    _write_recording(path, a, stats=stats)
    result = report.analyze(path)
    assert result["checks"]["A4 no I2C errors"] is False


def _tapped(axis, n=6000, amp=0.2, gravity_axis=2):
    rng = np.random.default_rng(3)
    t = np.arange(n) / 1000 - 2.0
    a = rng.normal(0, 0.001, size=(n, 3))
    a[:, gravity_axis] += 1.0
    ring = amp * np.exp(-0.04 * 2 * np.pi * 45 * t) * np.sin(2 * np.pi * 45 * t)
    a[:, axis] += np.where(t >= 0, ring, 0)
    return a


def test_load_scales_the_gyro_to_degrees_per_second(tmp_path):
    # The firmware sets +-500 dps, where one degree per second reads 65.5 counts.
    n = 100
    a = np.tile([0.0, 0.0, 1.0], (n, 1))
    gyro = np.zeros((n, 3), dtype=int)
    gyro[:, 2] = 655
    path = str(tmp_path / "rec")
    _write_recording(path, a, gyro_counts=gyro)
    assert analysis.load(path).gyro_dps[0, 2] == pytest.approx(10.0)


@pytest.mark.parametrize("axis,kind", [(0, "x"), (1, "y")])
def test_report_names_the_axis_of_a_tap(tmp_path, axis, kind):
    path = str(tmp_path / "rec")
    _write_recording(path, _tapped(axis))
    result = report.analyze(path)
    assert result["taps"][0]["kind"] == kind
    assert abs(result["direction"]["tilt_deg"]) < 1
    assert os.path.exists(os.path.join(path, "direction.png"))
    assert kind in report.summary(result)


def test_report_separates_a_twist_from_a_tap(tmp_path):
    n = 6000
    t = np.arange(n) / 1000 - 2.0
    ring = 5.0 * np.exp(-0.04 * 2 * np.pi * 45 * t) * np.sin(2 * np.pi * 45 * t)
    gyro = np.zeros((n, 3), dtype=int)
    gyro[:, 2] = np.round(np.where(t >= 0, ring, 0) * 65.5).astype(int)
    path = str(tmp_path / "rec")
    _write_recording(path, _tapped(0, amp=0.05), gyro_counts=gyro)
    result = report.analyze(path)
    assert result["taps"][0]["kind"] == "twist"
    assert result["direction"]["twist_peak_dps"] == pytest.approx(4.5, rel=0.2)


def test_report_survives_a_module_mounted_on_its_side(tmp_path):
    # Gravity on X: A2 still holds, but "along the files" means nothing for this mounting.
    path = str(tmp_path / "rec")
    _write_recording(path, _tapped(1, gravity_axis=0))
    result = report.analyze(path)
    assert result["direction"]["axis_deg"] is None
    assert result["taps"][0]["kind"] is None
    assert "not flat" in report.summary(result)


def test_report_separates_the_nine_events_of_a8(tmp_path):
    # Three strikes along X, three along Y, three twists, as the runbook's A8 asks for.
    n = 22000
    fs = 1000
    rng = np.random.default_rng(5)
    a = rng.normal(0, 0.001, size=(n, 3))
    a[:, 2] += 1.0
    gyro = rng.normal(0, 0.05, size=(n, 3))
    expected = ["x"] * 3 + ["y"] * 3 + ["twist"] * 3
    for i, kind in enumerate(expected):
        t = np.arange(n - (i + 1) * 2 * fs) / fs
        ring = np.exp(-0.04 * 2 * np.pi * 45 * t) * np.sin(2 * np.pi * 45 * t)
        start = (i + 1) * 2 * fs
        if kind == "twist":
            a[start:, 0] += 0.03 * ring
            gyro[start:, 2] += 3.0 * ring
        else:
            a[start:, 0 if kind == "x" else 1] += 0.2 * ring
    path = str(tmp_path / "rec")
    _write_recording(path, a, gyro_counts=np.round(gyro * 65.5).astype(int))
    result = report.analyze(path)
    assert [t["kind"] for t in result["taps"]] == expected
