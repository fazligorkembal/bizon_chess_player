"""Turn one recording into analysis.json, plots and a short verdict against tests A2-A3.

The thresholds are the stage-A acceptance criteria in the design doc, section 6; they are
written out here once so the printed PASS/FAIL means exactly what the doc means.
"""

import json
import os

import matplotlib

matplotlib.use("Agg")  # no display on a bench machine reached over SSH, and none needed
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from . import analysis  # noqa: E402

GRAVITY_TOLERANCE = 0.05    # A2: |a| within 1 g +- 5 %
INTERVAL_TOLERANCE = 0.02   # A3: mean interval within 1/rate +- 2 %


def analyze(path, noise_from=None):
    rec = analysis.load(path)
    fs = float(rec.meta["imu"]["rate"])
    result = {"label": rec.meta.get("label"), "rate_hz": fs, "stats": rec.meta.get("stats", {})}
    result["integrity"] = analysis.integrity(rec.seq, rec.t_us)
    g, magnitude, axis = analysis.gravity(rec.accel_g)
    result["gravity"] = {"vector_g": g.tolist(), "magnitude_g": magnitude,
                         "vertical_axis": "xyz"[axis]}
    h = analysis.horizontal(rec.accel_g, g)
    if noise_from:
        ref = analysis.load(noise_from)
        gr, _, _ = analysis.gravity(ref.accel_g)
        noise = analysis.noise_rms(analysis.horizontal(ref.accel_g, gr))
        result["noise_source"] = noise_from
    else:
        noise = analysis.robust_noise(h)
        result["noise_source"] = "robust estimate from this recording"
    result["noise_rms_g"] = noise
    result["horizontal_rms_g"] = analysis.noise_rms(h)
    taps = analysis.detect_taps(h, fs, noise)
    responses = [analysis.tap_response(h, fs, i, noise) for i in taps]
    result["taps"] = [{"t_s": float(rec.t_s[r.index]), "peak_g": r.peak_g, "freq_hz": r.freq_hz,
                       "damping": r.damping} for r in responses]
    result["marks"] = [{"name": n, "to_next_sample_us": d}
                       for n, d in analysis.mark_offsets_us(rec.events, rec.t_us)]
    result["checks"] = _checks(result, fs)
    with open(os.path.join(path, "analysis.json"), "w", encoding="utf-8") as fh:
        json.dump(result, fh, indent=2)
    _plots(path, rec, h, fs, noise, responses)
    return result


def _checks(result, fs):
    integ = result["integrity"]
    expected_us = 1e6 / fs
    return {
        "A2 gravity within 5%": abs(result["gravity"]["magnitude_g"] - 1.0) <= GRAVITY_TOLERANCE,
        "A3 no missing samples": integ["missing"] == 0 and integ["out_of_order"] == 0,
        "A3 interval within 2%":
            abs(integ["interval_mean_us"] - expected_us) <= INTERVAL_TOLERANCE * expected_us,
        "A4 no I2C errors": str(result["stats"].get("i2c_err", "0")) == "0",
    }


def _plots(path, rec, h, fs, noise, responses):
    mag = np.hypot(h[:, 0], h[:, 1])
    fig, ax = plt.subplots(figsize=(10, 3.5))
    ax.plot(rec.t_s, mag * 1000, lw=0.5)
    ax.axhline(10 * noise * 1000, color="tab:red", lw=0.8, label="tap threshold (10 x noise)")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("horizontal |a| [mg]")
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(os.path.join(path, "horizontal.png"), dpi=120)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(6, 3.5))
    ax.hist(np.diff(rec.t_us), bins=100)
    ax.set_xlabel("sample interval [us]")
    ax.set_ylabel("count")
    ax.set_yscale("log")
    fig.tight_layout()
    fig.savefig(os.path.join(path, "intervals.png"), dpi=120)
    plt.close(fig)

    f, p = analysis.psd(h, fs)
    fig, ax = plt.subplots(figsize=(8, 3.5))
    ax.semilogy(f, np.maximum(p, 1e-20))  # a perfectly still axis has zero power
    for r in responses:
        ax.axvline(r.freq_hz, color="tab:orange", lw=0.6)
    ax.set_xlabel("frequency [Hz]")
    ax.set_ylabel("PSD [g^2/Hz]")
    fig.tight_layout()
    fig.savefig(os.path.join(path, "psd.png"), dpi=120)
    plt.close(fig)


def summary(result):
    lines = [f"{result['label']}: {result['integrity']['samples']} samples"
             f" at {result['rate_hz']:.0f} Hz"]
    i = result["integrity"]
    lines.append(f"  missing {i['missing']}, out of order {i['out_of_order']}, interval "
                 f"{i['interval_mean_us']:.1f} us mean, {i['interval_std_us']:.1f} std, "
                 f"{i['interval_min_us']:.0f}..{i['interval_max_us']:.0f}")
    g = result["gravity"]
    lines.append(f"  gravity {g['magnitude_g']:.4f} g, vertical axis {g['vertical_axis']}")
    lines.append(f"  horizontal noise {result['noise_rms_g'] * 1000:.3f} mg rms"
                 f" ({result['noise_source']})")
    for t in result["taps"]:
        lines.append(f"  tap at {t['t_s']:.2f} s: peak {t['peak_g'] * 1000:.1f} mg, "
                     f"{t['freq_hz']:.1f} Hz, damping {t['damping']:.3f}")
    for m in result["marks"]:
        lines.append(f"  {m['name']}: next sample {m['to_next_sample_us']} us later")
    for name, ok in result["checks"].items():
        lines.append(f"  {'PASS' if ok else 'FAIL'}  {name}")
    return "\n".join(lines)
