import os

import numpy as np
import yaml

from joint_bench import report


def _write_recording(path, a_g, lsb=8192, rate=1000, drop=()):
    os.makedirs(path)
    n = len(a_g)
    seq = np.array([i for i in range(n + len(drop)) if i not in set(drop)])[:n]
    counts = np.round(a_g * lsb).astype(int)
    with open(os.path.join(path, "raw.csv"), "w") as fh:
        fh.write("seq,t_us,ax,ay,az,gx,gy,gz\n")
        for s, c in zip(seq, counts):
            fh.write(f"{s},{s * 1000},{c[0]},{c[1]},{c[2]},0,0,0\n")
    with open(os.path.join(path, "events.csv"), "w") as fh:
        fh.write("t_us,name\n2300,mark:a7-1\n")
    meta = {"label": "synthetic", "imu": {"rate": str(rate), "lsb_per_g": str(lsb)},
            "stats": {"i2c_err": "0"}}
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
