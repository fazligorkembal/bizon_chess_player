import numpy as np
import pytest

from joint_bench import analysis

FS = 1000.0


def _rest(n=20000, sigma=0.002, seed=1):
    rng = np.random.default_rng(seed)
    a = rng.normal(0.0, sigma, size=(n, 3))
    a[:, 2] += 1.0  # gravity on z
    return a


def test_integrity_counts_missing_and_out_of_order_samples():
    seq = np.array([0, 1, 2, 5, 6, 6, 7])
    t_us = np.array([0, 1000, 2000, 5000, 6000, 6000, 7000])
    r = analysis.integrity(seq, t_us)
    assert r["missing"] == 2
    assert r["out_of_order"] == 1
    assert r["interval_max_us"] == 3000


def test_gravity_finds_one_g_on_the_vertical_axis_despite_taps():
    a = _rest()
    a[5000:5050, 0] += 0.8  # a tap must not drag the static estimate
    g, magnitude, axis = analysis.gravity(a)
    assert magnitude == pytest.approx(1.0, abs=0.005)
    assert axis == 2


def test_horizontal_removes_gravity_and_keeps_platform_motion():
    a = _rest(sigma=0.0)
    a[:, 0] += 0.1 * np.sin(np.arange(len(a)) / FS * 2 * np.pi * 10)
    g, _, _ = analysis.gravity(a)
    h = analysis.horizontal(a, np.array([0.0, 0.0, 1.0]))
    assert np.max(np.abs(np.hypot(h[:, 0], h[:, 1]))) == pytest.approx(0.1, rel=0.01)


def test_tilted_mounting_still_yields_horizontal_motion_only():
    tilt = np.deg2rad(10)
    g = np.array([np.sin(tilt), 0.0, np.cos(tilt)])
    a = np.tile(g, (1000, 1))
    assert np.max(np.abs(analysis.horizontal(a, g))) < 1e-12


def test_robust_noise_matches_the_true_noise_and_ignores_taps():
    sigma = 0.002
    a = _rest(sigma=sigma)
    g, _, _ = analysis.gravity(a)
    h = analysis.horizontal(a, g)
    clean = analysis.noise_rms(h)
    assert clean == pytest.approx(sigma * np.sqrt(2), rel=0.05)
    h[1000:1100] += 0.5
    assert analysis.robust_noise(h) == pytest.approx(clean, rel=0.05)


def test_psd_peaks_at_the_injected_frequency():
    t = np.arange(20000) / FS
    h = np.column_stack([0.01 * np.sin(2 * np.pi * 63 * t), np.zeros_like(t)])
    f, p = analysis.psd(h, FS)
    assert f[np.argmax(p)] == pytest.approx(63, abs=0.5)


def _tapped(f0, zeta, taps_at, noise=0.0005, n=10000, seed=2):
    rng = np.random.default_rng(seed)
    h = rng.normal(0.0, noise / np.sqrt(2), size=(n, 2))
    t = np.arange(n) / FS
    direction = np.array([0.6, 0.8])
    for t0 in taps_at:
        tt = t - t0
        decay = np.exp(-zeta * 2 * np.pi * f0 * tt)
        ring = np.where(tt >= 0, 0.3 * decay * np.sin(2 * np.pi * f0 * tt), 0)
        h += np.outer(ring, direction)
    return h


def test_detects_each_tap_once():
    h = _tapped(37.0, 0.03, taps_at=[1.0, 3.0, 5.5])
    taps = analysis.detect_taps(h, FS, noise=0.0005)
    assert [round(i / FS, 1) for i in taps] == [1.0, 3.0, 5.5]


def test_recovers_frequency_and_damping_of_the_ringing():
    h = _tapped(37.0, 0.03, taps_at=[1.0])
    tap = analysis.detect_taps(h, FS, noise=0.0005)[0]
    r = analysis.tap_response(h, FS, tap, noise=0.0005)
    assert r.freq_hz == pytest.approx(37.0, abs=0.5)
    assert r.damping == pytest.approx(0.03, rel=0.15)


def test_mark_offsets_measure_time_to_the_next_sample():
    t_us = np.arange(0, 10000, 1000)
    events = [(2300, "mark:a"), (5000, "mark"), (7000, "other")]
    assert analysis.mark_offsets_us(events, t_us) == [("mark:a", 700), ("mark", 0)]


def test_load_refuses_an_interrupted_recording(tmp_path):
    (tmp_path / "raw.csv").write_text("seq,t_us,ax,ay,az,gx,gy,gz\n0,0,0,0,8192,0,0,0\n")
    with pytest.raises(FileNotFoundError, match="interrupted"):
        analysis.load(str(tmp_path))
