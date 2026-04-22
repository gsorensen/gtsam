#!/usr/bin/env python3
"""IMU characterisation for the IFAC WC 2026 dataset.

Reads df_flat_data.csv and produces:
  1. Raw gyro/accel spectra on a static window (pre-takeoff) and a cruise
     window. Peaks well above the static floor are mechanical vibration.
  2. Overlapping Allan variance on the static window, per gyro and accel
     axis. Extracts:
        - ARW  (angle random walk, rad/sqrt(s))                -- slope -1/2
        - RRW  (rate random walk, rad/s/sqrt(s))               -- slope +1/2
        - Bias instability (rad/s)                              -- flat min
     These map directly to the sigmas consumed by GTSAM's PreintegrationParams.
  3. Optional bias-observability view (if --debug is supplied): overlays
     bg_z(t), truth yaw-rate, and yaw-error on a shared axis.

Static window is auto-detected as the earliest contiguous window with
low gyro magnitude. Override with --static-start / --static-end (seconds).

Usage:
  python characterise_imu.py \
      --data ~/ws/ntnu/parnav/parnav-scripts/post_processing/df_flat_data.csv \
      [--debug <debug.csv>] [--truth <truth.csv>] \
      [--out-dir <dir>]
"""
from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

DEFAULT_DIR = Path.home() / "ws/ntnu/parnav/parnav-scripts/post_processing"


# ---------------------------------------------------------------------------
# Static-window auto-detection
# ---------------------------------------------------------------------------

def find_static_window(t, w_m, min_s=20.0, stride_s=1.0):
    """Return (t_start, t_end) of lowest-gyro-magnitude contiguous window.

    Uses a sliding-window RMS of |w_m|. The window of length min_s with the
    lowest RMS is picked. Stride is stride_s (coarse) -- good enough for
    auto-pick; user can override with --static-start/--static-end.
    """
    dt = np.median(np.diff(t))
    nwin = int(round(min_s / dt))
    nstride = max(1, int(round(stride_s / dt)))
    mag = np.linalg.norm(w_m, axis=0)
    best_rms = np.inf
    best_i0 = 0
    for i0 in range(0, len(mag) - nwin, nstride):
        rms = np.sqrt(np.mean(mag[i0:i0 + nwin] ** 2))
        if rms < best_rms:
            best_rms = rms
            best_i0 = i0
    return t[best_i0], t[best_i0 + nwin - 1], best_rms


# ---------------------------------------------------------------------------
# Spectra
# ---------------------------------------------------------------------------

def one_sided_psd(x, fs):
    """Simple one-sided PSD with Hann window. Returns (f, Pxx)."""
    x = x - x.mean()
    win = np.hanning(len(x))
    X = np.fft.rfft(x * win)
    # Normalisation: PSD in units^2 / Hz.
    scale = 2.0 / (fs * (win ** 2).sum())
    Pxx = (np.abs(X) ** 2) * scale
    f = np.fft.rfftfreq(len(x), 1.0 / fs)
    return f, Pxx


def plot_spectra(t, w_m, f_m, static_t, cruise_t, out_dir):
    dt = np.median(np.diff(t))
    fs = 1.0 / dt

    def slice_window(arr, window):
        mask = (t >= window[0]) & (t <= window[1])
        return arr[:, mask]

    fig, axes = plt.subplots(2, 3, figsize=(14, 7), sharex=True)
    names_g = ["gyro_x", "gyro_y", "gyro_z"]
    names_a = ["accel_x", "accel_y", "accel_z"]

    for col, label in enumerate(names_g):
        ax = axes[0, col]
        for win_t, lab, ls in [(static_t, "static", "-"),
                               (cruise_t, "cruise", "-")]:
            w = slice_window(w_m, win_t)
            if w.shape[1] < 64:
                continue
            f, P = one_sided_psd(w[col], fs)
            ax.loglog(f, np.sqrt(P), ls, label=lab, lw=0.7)
        ax.set_title(label)
        ax.grid(True, which="both", lw=0.3)
        if col == 0:
            ax.set_ylabel(r"gyro ASD [rad/s/$\sqrt{Hz}$]")
        ax.legend(fontsize=8)

    for col, label in enumerate(names_a):
        ax = axes[1, col]
        for win_t, lab in [(static_t, "static"), (cruise_t, "cruise")]:
            w = slice_window(f_m, win_t)
            if w.shape[1] < 64:
                continue
            f, P = one_sided_psd(w[col], fs)
            ax.loglog(f, np.sqrt(P), label=lab, lw=0.7)
        ax.set_title(label)
        ax.set_xlabel("f [Hz]")
        ax.grid(True, which="both", lw=0.3)
        if col == 0:
            ax.set_ylabel(r"accel ASD [m/s$^2$/$\sqrt{Hz}$]")
        ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(out_dir / "imu_spectra.png", dpi=140)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Allan variance
# ---------------------------------------------------------------------------

def allan_variance(x, fs, n_tau=60):
    """Overlapping Allan variance for a 1-D signal x sampled at fs Hz.

    Returns (tau, adev) with adev = sigma_y(tau) in the same units as x.
    """
    N = len(x)
    # log-spaced m (samples per cluster)
    m_max = N // 4
    ms = np.unique(np.logspace(0, np.log10(m_max), n_tau).astype(int))
    taus = ms / fs
    adev = np.empty_like(taus, dtype=float)

    # Cumulative theta (integrated signal).
    theta = np.cumsum(x) / fs
    for i, m in enumerate(ms):
        # Overlapping sample mean differences.
        d = theta[2 * m:] - 2 * theta[m:-m] + theta[:-2 * m]
        var = np.mean(d ** 2) / (2 * (m / fs) ** 2)
        adev[i] = np.sqrt(var)
    return taus, adev


def fit_allan_params(tau, adev):
    """Crude ARW / RRW / bias-instability estimates.

    ARW = adev(1s) (assumes slope -1/2 near tau=1s).
    RRW = adev(tau_max) * sqrt(3/tau_max)   (slope +1/2 region).
    BI  = min(adev) * sqrt(2 / (2*ln(2)/pi)) ≈ min(adev) / 0.664.
    """
    # Nearest tau to 1s for ARW.
    i1 = int(np.argmin(np.abs(tau - 1.0)))
    arw = adev[i1] * np.sqrt(tau[i1])  # rad/sqrt(s): adev*sqrt(tau) is flat for WN
    # Actually: for angle random walk N, adev(tau) = N / sqrt(tau).
    # So N = adev * sqrt(tau). Take it at tau=1.
    # RRW: adev(tau) = K * sqrt(tau/3) => K = adev * sqrt(3/tau) at large tau
    i_max = len(tau) - 1
    rrw = adev[i_max] * np.sqrt(3.0 / tau[i_max])
    # Bias instability (flat floor): min(adev)*pi/(2*ln(2)) factor.
    bi = adev.min() / 0.664
    return arw, rrw, bi


def plot_allan(w_m_static, f_m_static, fs, out_dir):
    fig, axes = plt.subplots(1, 2, figsize=(12, 5))
    params = {"gyro": [], "accel": []}

    for col, (name, unit) in enumerate([
        ("gyro",  r"$\sigma_y$ [rad/s]"),
        ("accel", r"$\sigma_y$ [m/s$^2$]"),
    ]):
        ax = axes[col]
        sig = w_m_static if name == "gyro" else f_m_static
        for axis_i, axis_n in enumerate(["x", "y", "z"]):
            tau, adev = allan_variance(sig[axis_i], fs)
            ax.loglog(tau, adev, label=f"{name}_{axis_n}", lw=0.8)
            arw, rrw, bi = fit_allan_params(tau, adev)
            params[name].append((axis_n, arw, rrw, bi))
        ax.set_xlabel(r"$\tau$ [s]")
        ax.set_ylabel(unit)
        ax.set_title(f"Allan deviation — {name}")
        ax.grid(True, which="both", lw=0.3)
        ax.legend(fontsize=8)

    fig.tight_layout()
    fig.savefig(out_dir / "allan_deviation.png", dpi=140)
    plt.close(fig)

    # Console report.
    print("\n=== Allan-variance-derived IMU params ===")
    print(f"{'axis':<8}{'ARW':>14}{'RRW':>16}{'BiasInstab':>18}")
    for name in ("gyro", "accel"):
        unit_arw = "rad/√s" if name == "gyro" else "(m/s²)/√s"
        unit_rrw = "rad/s/√s" if name == "gyro" else "(m/s²)/s/√s ... effectively bias walk"
        for axis_n, arw, rrw, bi in params[name]:
            print(f"{name}_{axis_n:<6}{arw:>14.4e}{rrw:>16.4e}{bi:>18.4e}")
    print("\nMap to GTSAM PreintegrationParams:")
    print("  gyroscopeCovariance      ~ (ARW_gyro)^2   per axis")
    print("  biasOmegaCovariance      ~ (RRW_gyro)^2   per axis")
    print("  accelerometerCovariance  ~ (ARW_accel)^2  per axis")
    print("  biasAccCovariance        ~ (RRW_accel)^2  per axis")
    return params


# ---------------------------------------------------------------------------
# Bias observability
# ---------------------------------------------------------------------------

def plot_bias_observability(debug_path, truth_path, out_dir):
    if not debug_path.exists():
        return
    dbg = pd.read_csv(debug_path)
    tru = pd.read_csv(truth_path) if truth_path.exists() else None
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)

    axes[0].plot(dbg.t, dbg.bg_z, lw=0.7, label="bg_z [rad/s]")
    axes[0].grid(True); axes[0].legend(); axes[0].set_ylabel("gyro-z bias")

    if tru is not None:
        # Nearest-neighbour truth yaw at debug times.
        idx = np.searchsorted(tru.t.to_numpy(), dbg.t.to_numpy())
        idx = np.clip(idx, 0, len(tru) - 1)
        yaw_tru = tru.yaw.to_numpy()[idx]
        yaw_tru_unwrap = np.unwrap(yaw_tru)
        yaw_rate = np.gradient(yaw_tru_unwrap, dbg.t.to_numpy())
        axes[1].plot(dbg.t, np.degrees(yaw_rate), lw=0.7, color="tab:orange")
        axes[1].set_ylabel("truth yaw rate [deg/s]")
        axes[1].grid(True)

        yaw_err = (dbg.yaw_post.to_numpy() - yaw_tru + np.pi) % (2 * np.pi) - np.pi
        axes[2].plot(dbg.t, np.degrees(yaw_err), lw=0.7, color="tab:red")
        axes[2].set_ylabel("yaw error [deg]")
        axes[2].grid(True)

        # Cross-correlation bg_z vs yaw-rate (lag 0).
        bg = dbg.bg_z.to_numpy()
        bg_c = (bg - bg.mean()) / (bg.std() + 1e-12)
        yr_c = (yaw_rate - yaw_rate.mean()) / (yaw_rate.std() + 1e-12)
        corr = float(np.mean(bg_c * yr_c))
        axes[0].set_title(f"corr(bg_z, yaw_rate_truth) = {corr:+.3f}   "
                          "(|>0.3| => vibration rectification / observability issue)")

    axes[-1].set_xlabel("t [s]")
    fig.tight_layout()
    fig.savefig(out_dir / "bias_observability.png", dpi=140)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", type=Path,
                    default=DEFAULT_DIR / "df_flat_data.csv")
    ap.add_argument("--debug", type=Path,
                    default=DEFAULT_DIR /
                    "ifac_wc_2026_se3_cb_no_handover_no_robust_debug.csv")
    ap.add_argument("--truth", type=Path,
                    default=DEFAULT_DIR / "df_flat_truth.csv")
    ap.add_argument("--out-dir", type=Path,
                    default=DEFAULT_DIR / "imu_characterisation")
    ap.add_argument("--static-start", type=float, default=None,
                    help="Override static-window start [s].")
    ap.add_argument("--static-end", type=float, default=None,
                    help="Override static-window end [s].")
    ap.add_argument("--cruise-start", type=float, default=None)
    ap.add_argument("--cruise-end", type=float, default=None)
    args = ap.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    print(f"Loading {args.data} ...")
    df = pd.read_csv(args.data)
    t = df.t.to_numpy()
    w_m = df[["w_m_1", "w_m_2", "w_m_3"]].to_numpy().T
    f_m = df[["f_m_1", "f_m_2", "f_m_3"]].to_numpy().T
    dt = np.median(np.diff(t))
    fs = 1.0 / dt
    print(f"  N={len(t)}, fs≈{fs:.1f} Hz, span=[{t[0]:.3f},{t[-1]:.3f}] s")

    # Static window.
    if args.static_start is not None and args.static_end is not None:
        static_t = (args.static_start, args.static_end)
        print(f"Static window (user): [{static_t[0]:.2f},{static_t[1]:.2f}] s")
    else:
        s0, s1, rms = find_static_window(t, w_m, min_s=20.0)
        static_t = (s0, s1)
        print(f"Static window (auto): [{s0:.2f},{s1:.2f}] s  "
              f"|w| rms={rms:.4e} rad/s")

    # Cruise window: default = 60s chunk from the middle of the flight.
    if args.cruise_start is not None and args.cruise_end is not None:
        cruise_t = (args.cruise_start, args.cruise_end)
    else:
        mid = 0.5 * (t[0] + t[-1])
        cruise_t = (mid - 30.0, mid + 30.0)
    print(f"Cruise window: [{cruise_t[0]:.2f},{cruise_t[1]:.2f}] s")

    plot_spectra(t, w_m, f_m, static_t, cruise_t, args.out_dir)

    # Allan on static window only.
    mask = (t >= static_t[0]) & (t <= static_t[1])
    w_s = w_m[:, mask]
    f_s = f_m[:, mask]
    plot_allan(w_s, f_s, fs, args.out_dir)

    plot_bias_observability(args.debug, args.truth, args.out_dir)

    print(f"\nPlots written to {args.out_dir}")


if __name__ == "__main__":
    main()
