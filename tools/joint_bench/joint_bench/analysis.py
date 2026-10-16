"""Analysis of one recording: sample integrity, gravity, noise, spectrum, taps, marks.

Everything here is a pure function of arrays, so it is tested on synthetic signals with
known answers. Accelerations are in g; times in seconds unless a name says otherwise.
"""

import csv
import os
from dataclasses import dataclass

import numpy as np
import yaml
from scipy import signal


@dataclass
class Recording:
    meta: dict
    seq: np.ndarray
    t_s: np.ndarray        # firmware time, seconds from the first sample
    t_us: np.ndarray       # firmware time, raw microseconds
    accel_g: np.ndarray    # shape (n, 3)
    events: list           # [(t_us, name)]


def load(path):
    meta_path = os.path.join(path, "meta.yaml")
    if not os.path.exists(meta_path):
        raise FileNotFoundError(f"{path} has no meta.yaml; the recording was interrupted")
    with open(meta_path, encoding="utf-8") as fh:
        meta = yaml.safe_load(fh)
    data = np.loadtxt(os.path.join(path, "raw.csv"), delimiter=",", skiprows=1,
                      dtype=np.int64, ndmin=2)
    lsb = float(meta["imu"]["lsb_per_g"])
    events = []
    with open(os.path.join(path, "events.csv"), encoding="ascii") as fh:
        for row in csv.DictReader(fh):
            events.append((int(row["t_us"]), row["name"]))
    t_us = data[:, 1]
    return Recording(meta=meta, seq=data[:, 0], t_us=t_us, t_s=(t_us - t_us[0]) / 1e6,
                     accel_g=data[:, 2:5] / lsb, events=events)


def integrity(seq, t_us):
    """Missing samples from seq gaps, and the sample-interval distribution in microseconds."""
    dseq = np.diff(seq)
    dt = np.diff(t_us).astype(float)
    return {
        "samples": int(len(seq)),
        "missing": int(np.sum(dseq[dseq > 1] - 1)),
        "out_of_order": int(np.sum(dseq < 1)),
        "interval_mean_us": float(dt.mean()) if len(dt) else float("nan"),
        "interval_std_us": float(dt.std()) if len(dt) else float("nan"),
        "interval_min_us": float(dt.min()) if len(dt) else float("nan"),
        "interval_max_us": float(dt.max()) if len(dt) else float("nan"),
    }


def gravity(accel_g):
    """Static acceleration from the per-axis median, so sparse taps do not bias it."""
    g = np.median(accel_g, axis=0)
    magnitude = float(np.linalg.norm(g))
    vertical_axis = int(np.argmax(np.abs(g)))
    return g, magnitude, vertical_axis


def horizontal_basis(g):
    """Two orthonormal vectors spanning the platform plane, the plane normal to gravity."""
    u = g / np.linalg.norm(g)
    helper = np.eye(3)[int(np.argmin(np.abs(u)))]
    e1 = np.cross(u, helper)
    e1 /= np.linalg.norm(e1)
    e2 = np.cross(u, e1)
    return e1, e2


def horizontal(accel_g, g):
    """(n, 2) horizontal components of the dynamic acceleration. Pieces slide and tip under
    horizontal acceleration, so this, not the raw vector, is what every metric uses."""
    e1, e2 = horizontal_basis(g)
    dyn = accel_g - g
    return np.column_stack([dyn @ e1, dyn @ e2])


def noise_rms(h):
    """RMS of the horizontal magnitude: the floor every later threshold is a multiple of."""
    return float(np.sqrt(np.mean(np.sum(h ** 2, axis=1))))


def robust_noise(h):
    """Noise estimate that sparse taps do not inflate: 1.4826 x MAD of each component,
    combined as an RMS magnitude. Equals noise_rms for pure Gaussian noise."""
    sigma = [1.4826 * np.median(np.abs(c - np.median(c))) for c in h.T]
    return float(np.sqrt(sum(s ** 2 for s in sigma)))


def psd(h, fs):
    """Welch PSD of each horizontal component, summed: g^2/Hz against Hz."""
    nperseg = int(min(len(h), 4 * fs))
    f, p1 = signal.welch(h[:, 0], fs=fs, nperseg=nperseg)
    _, p2 = signal.welch(h[:, 1], fs=fs, nperseg=nperseg)
    return f, p1 + p2


def detect_taps(h, fs, noise, k=10.0, quiet_s=0.1):
    """Indices where |h| crosses k x noise. After a tap the detector re-arms only once |h|
    has stayed below the threshold for quiet_s: a lightly damped platform rings above it
    for most of a second, and without the hysteresis one strike would count as several."""
    mag = np.hypot(h[:, 0], h[:, 1])
    threshold = k * noise
    quiet_needed = int(quiet_s * fs)
    taps = []
    armed = True
    quiet = 0
    for i, m in enumerate(mag):
        if m > threshold:
            quiet = 0
            if armed:
                taps.append(i)
                armed = False
        else:
            quiet += 1
            if quiet >= quiet_needed:
                armed = True
    return taps


@dataclass
class TapResponse:
    index: int
    peak_g: float
    freq_hz: float
    damping: float  # damping ratio zeta; nan when the decay could not be fitted


def tap_response(h, fs, index, noise, window_s=0.5):
    """Dominant frequency and damping ratio of the ringing after one tap.

    The ringing is projected onto its own principal horizontal direction first: the
    magnitude |h| is rectified and would show every frequency doubled.
    """
    seg = h[index:index + int(window_s * fs)]
    seg = seg - seg.mean(axis=0)
    _, _, vt = np.linalg.svd(seg, full_matrices=False)
    s = seg @ vt[0]
    peak = int(np.argmax(np.abs(s)))
    s = s[peak:]
    n = len(s)
    spec = np.abs(np.fft.rfft(s * np.hanning(n), n=8 * n))
    freqs = np.fft.rfftfreq(8 * n, 1 / fs)
    band = (freqs >= 2.0) & (freqs <= 0.45 * fs)
    f0 = float(freqs[band][np.argmax(spec[band])])
    damping = _damping(s, fs, f0, noise)
    return TapResponse(index=index, peak_g=float(np.max(np.abs(seg))), freq_hz=f0, damping=damping)


def _damping(s, fs, f0, noise):
    # Envelope of the band around f0, fitted as exp(-zeta * 2 pi f0 * t) while it is still
    # well above the noise. Below 3 x noise the fit would be measuring the noise floor.
    lo, hi = 0.7 * f0, min(1.3 * f0, 0.45 * fs)
    sos = signal.butter(2, [lo, hi], btype="band", fs=fs, output="sos")
    env = np.abs(signal.hilbert(signal.sosfiltfilt(sos, s)))
    usable = np.flatnonzero(env < 3 * noise)
    end = int(usable[0]) if len(usable) else len(env)
    start = int(round(fs / f0))  # skip the first cycle, where the filter is settling
    if end - start < int(2 * fs / f0):
        return float("nan")
    t = np.arange(start, end) / fs
    slope = np.polyfit(t, np.log(env[start:end]), 1)[0]
    return float(-slope / (2 * np.pi * f0))


def mark_offsets_us(events, t_us):
    """For each mark event, microseconds until the next sample. Compared in test A7 with
    the MARK-edge-to-INT-edge interval read off the logic analyzer."""
    out = []
    for t, name in events:
        if not name.startswith("mark"):
            continue
        after = t_us[t_us >= t]
        if len(after):
            out.append((name, int(after[0] - t)))
    return out
