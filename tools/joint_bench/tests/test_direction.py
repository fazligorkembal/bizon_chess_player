import numpy as np
import pytest

from joint_bench import analysis

FS = 1000.0


def _sine(n=2000, f=10.0, amp=0.1):
    return amp * np.sin(2 * np.pi * f * np.arange(n) / FS)


def _ring(n, f=45.0, amp=0.2):
    t = np.arange(n) / FS
    return amp * np.exp(-8 * t) * np.sin(2 * np.pi * f * t)


def test_tilt_is_zero_when_flat():
    g = [0, 0, 1]
    result = analysis.tilt(g)
    assert result["tilt_deg"] == pytest.approx(0, abs=1e-9)
    assert result["roll_deg"] == pytest.approx(0, abs=1e-9)
    assert result["pitch_deg"] == pytest.approx(0, abs=1e-9)


def test_tilt_reports_roll():
    a = np.deg2rad(10)
    g = [0, np.sin(a), np.cos(a)]
    result = analysis.tilt(g)
    assert result["tilt_deg"] == pytest.approx(10, abs=1e-6)
    assert result["roll_deg"] == pytest.approx(10, abs=1e-6)
    assert result["pitch_deg"] == pytest.approx(0, abs=1e-6)


def test_tilt_reports_pitch():
    a = np.deg2rad(10)
    g = [-np.sin(a), 0, np.cos(a)]
    result = analysis.tilt(g)
    assert result["tilt_deg"] == pytest.approx(10, abs=1e-6)
    assert result["pitch_deg"] == pytest.approx(10, abs=1e-6)
    assert result["roll_deg"] == pytest.approx(0, abs=1e-6)


def test_tilt_of_an_upside_down_module_is_zero():
    g = [0, 0, -1]
    result = analysis.tilt(g)
    assert result["tilt_deg"] == pytest.approx(0, abs=1e-9)


def test_platform_axes_match_the_sensor_axes_when_flat():
    g = [0, 0, 1]
    ex, ey = analysis.platform_axes(g)
    assert ex == pytest.approx([1, 0, 0], abs=1e-9)
    assert ey == pytest.approx([0, 1, 0], abs=1e-9)


def test_platform_axes_are_orthonormal_when_tilted():
    g = [0.1, 0.1, 0.99]
    ex, ey = analysis.platform_axes(g)
    assert np.dot(ex, ey) == pytest.approx(0, abs=1e-9)
    assert np.dot(ex, g) == pytest.approx(0, abs=1e-9)
    assert np.dot(ey, g) == pytest.approx(0, abs=1e-9)
    assert np.linalg.norm(ex) == pytest.approx(1, abs=1e-9)
    assert np.linalg.norm(ey) == pytest.approx(1, abs=1e-9)


def test_platform_axes_refuse_a_vertical_x_axis():
    g = [1, 0, 0]
    with pytest.raises(ValueError):
        analysis.platform_axes(g)


def test_platform_xy_removes_gravity():
    g = [0, 0, 1]
    a = np.tile(g, (100, 1))
    xy = analysis.platform_xy(a, g)
    assert xy.shape == (100, 2)
    assert np.allclose(xy, 0, atol=1e-12)


def test_principal_axis_along_x():
    rng = np.random.default_rng(1)
    x = _sine() + rng.normal(0, 0.0005, 2000)
    y = rng.normal(0, 0.0005, 2000)
    xy = np.column_stack([x, y])
    result = analysis.principal_axis(xy)
    assert abs(result.axis_deg) < 2
    assert result.anisotropy > 20
    assert result.major_rms_g == pytest.approx(0.0707, rel=0.05)


def test_principal_axis_along_y():
    rng = np.random.default_rng(1)
    x = rng.normal(0, 0.0005, 2000)
    y = _sine() + rng.normal(0, 0.0005, 2000)
    xy = np.column_stack([x, y])
    result = analysis.principal_axis(xy)
    assert abs(abs(result.axis_deg) - 90) < 2


def test_principal_axis_at_thirty_degrees():
    s = _sine()
    xy = np.column_stack([np.cos(np.deg2rad(30)) * s, np.sin(np.deg2rad(30)) * s])
    result = analysis.principal_axis(xy)
    assert result.axis_deg == pytest.approx(30, abs=1)


def test_principal_axis_at_minus_forty_five_degrees():
    s = _sine()
    xy = np.column_stack([np.cos(np.deg2rad(-45)) * s, np.sin(np.deg2rad(-45)) * s])
    result = analysis.principal_axis(xy)
    assert result.axis_deg == pytest.approx(-45, abs=1)


def test_principal_axis_ignores_the_sign_of_the_motion():
    s = _sine()
    xy1 = np.column_stack([np.cos(np.deg2rad(30)) * s, np.sin(np.deg2rad(30)) * s])
    xy2 = np.column_stack([-np.cos(np.deg2rad(30)) * s, -np.sin(np.deg2rad(30)) * s])
    result1 = analysis.principal_axis(xy1)
    result2 = analysis.principal_axis(xy2)
    assert result1.axis_deg == pytest.approx(result2.axis_deg, abs=1e-6)


def test_isotropic_noise_has_no_clear_axis():
    rng = np.random.default_rng(1)
    x = rng.normal(0, 0.01, 20000)
    y = rng.normal(0, 0.01, 20000)
    xy = np.column_stack([x, y])
    result = analysis.principal_axis(xy)
    assert result.anisotropy < 1.2


def test_anisotropy_is_capped_for_a_perfect_line():
    x = _sine()
    y = np.zeros_like(x)
    xy = np.column_stack([x, y])
    result = analysis.principal_axis(xy)
    assert result.anisotropy == 1000.0


def test_twist_rate_removes_the_bias():
    n = 2000
    gyro = np.zeros((n, 3))
    gyro[:, 2] = 2.0 + _sine(amp=50.0)
    g = [0, 0, 1]
    result = analysis.twist_rate(gyro, g)
    assert np.max(np.abs(result - _sine(amp=50.0))) < 0.5


def test_twist_rate_ignores_the_horizontal_axes():
    n = 2000
    gyro = np.zeros((n, 3))
    gyro[:, 0] = _sine(amp=100.0)
    g = [0, 0, 1]
    result = analysis.twist_rate(gyro, g)
    assert np.max(np.abs(result)) < 1e-9


def test_twist_rate_follows_the_direction_of_gravity():
    n = 2000
    gyro = np.zeros((n, 3))
    gyro[:, 2] = _sine(amp=50.0)
    g = [0, 0, -1]
    result = analysis.twist_rate(gyro, g)
    assert np.max(np.abs(result + _sine(amp=50.0))) < 0.5


@pytest.mark.parametrize("angle, kind", [(0, "x"), (25, "x"), (-25, "x"), (90, "y"), (-80, "y"), (65, "y"),
                                         (45, "diagonal"), (-45, "diagonal")])
def test_classify_event_names_the_axis(angle, kind):
    assert analysis.classify_event(angle, 10.0, 0.2, 0.1) == kind


def test_classify_event_without_a_clear_axis():
    assert analysis.classify_event(0, 1.5, 0.2, 0.1) == "unclear"


def test_classify_event_finds_a_twist():
    assert analysis.classify_event(0, 10.0, 0.2, 5.0) == "twist"


def test_classify_event_keeps_a_small_twist_as_a_tap():
    assert analysis.classify_event(0, 10.0, 0.2, 0.5) == "x"


def test_classify_event_survives_no_motion():
    assert analysis.classify_event(0, 1.0, 0.0, 0.0) == "unclear"


def test_event_direction_finds_a_ringing_along_y():
    n = 2000
    xy = np.zeros((n, 2))
    xy[500:, 1] = _ring(1500)
    twist = np.zeros(n)
    e = analysis.event_direction(xy, twist, FS, 500)
    assert e.kind == "y"
    assert abs(abs(e.axis_deg) - 90) < 3
    assert e.peak_g == pytest.approx(0.19, rel=0.1)
    assert e.twist_peak_dps == 0


def test_event_direction_uses_only_its_window():
    n = 2000
    xy = np.zeros((n, 2))
    xy[500:, 0] = _ring(1500)
    xy[1500:, 1] += 0.5
    twist = np.zeros(n)
    e = analysis.event_direction(xy, twist, FS, 500)
    assert e.kind == "x"


def test_event_direction_finds_a_twist():
    n = 2000
    xy = np.zeros((n, 2))
    xy[500:, 0] = _ring(1500, amp=0.05)
    twist = np.zeros(n)
    twist[500:] = _ring(1500, amp=5.0)
    e = analysis.event_direction(xy, twist, FS, 500)
    assert e.kind == "twist"
    assert e.twist_peak_dps == pytest.approx(4.8, rel=0.1)