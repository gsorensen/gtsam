#!/usr/bin/env python3
"""Save one 3-sigma error plot PER CASE for the RMSE matrix (no display).

One 5x3 figure (5 states x 3 axes) per manifest row (per run): that run's
error line with its +/-3sigma envelope. Saved as PNG; nothing shown.

Filename: <asmp>_<pose>_<bias>_<method>.png, e.g. simple_se23_gm_our-full.
The method is included because an (asmp,pose,bias) triple has up to four SE23
methods, which would otherwise collide.
Usage: plot_rmse_matrix_sigma.py <manifest.tsv> <out_dir>
"""
import os
import re
import sys
from collections import OrderedDict

import matplotlib
matplotlib.use("Agg")  # headless: save only, never display
import matplotlib.pyplot as plt  # noqa: E402
import pandas as pd  # noqa: E402

# state -> (err cols, sig3 cols, axis labels)
STATES = OrderedDict([
    ("Att",  (["att_err_roll", "att_err_pitch", "att_err_yaw"],
              ["sig3_roll", "sig3_pitch", "sig3_yaw"], ["roll", "pitch", "yaw"])),
    ("Pos",  (["pos_err_n", "pos_err_e", "pos_err_d"],
              ["sig3_pos_n", "sig3_pos_e", "sig3_pos_d"], ["N", "E", "D"])),
    ("Vel",  (["vel_err_n", "vel_err_e", "vel_err_d"],
              ["sig3_vel_n", "sig3_vel_e", "sig3_vel_d"], ["N", "E", "D"])),
    ("AccB", (["acc_bias_err_x", "acc_bias_err_y", "acc_bias_err_z"],
              ["sig3_ab_x", "sig3_ab_y", "sig3_ab_z"], ["x", "y", "z"])),
    ("GyrB", (["gyro_bias_err_x", "gyro_bias_err_y", "gyro_bias_err_z"],
              ["sig3_gb_x", "sig3_gb_y", "sig3_gb_z"], ["x", "y", "z"])),
])


def slug(s):
    return re.sub(r"[^a-z0-9]+", "-", s.lower()).strip("-")


def plot_case(mode, asmp, pose, bias, method, csv, outdir):
    """One 5x3 figure for a single run: its error + /-3sigma envelope."""
    try:
        df = pd.read_csv(csv)
    except Exception:
        print(f"skip (no csv): {csv}")
        return
    t = df["t"].to_numpy()
    fig, axes = plt.subplots(5, 3, figsize=(14, 16), sharex=True)
    for si, (state, (ec, sc, axl)) in enumerate(STATES.items()):
        for ai in range(3):
            ax = axes[si, ai]
            ax.plot(t, df[ec[ai]], color="tab:blue", lw=0.8, label="error")
            s3 = df[sc[ai]].to_numpy()
            ax.plot(t, s3, color="tab:red", lw=0.6, ls="--", label="+/-3sigma")
            ax.plot(t, -s3, color="tab:red", lw=0.6, ls="--")
            ax.set_title(f"{state} {axl[ai]}", fontsize=9)
            ax.grid(True, lw=0.3)
    axes[0, 0].legend(fontsize=7, loc="upper left")
    for ax in axes[-1]:
        ax.set_xlabel("t [s]")
    name = f"{slug(mode)}_{slug(asmp)}_{slug(pose)}_{slug(bias)}_{slug(method)}"
    fig.suptitle(f"{mode} / {asmp} / {pose}-{bias.upper()}-{method}"
                 " — error +/- 3sigma")
    fig.tight_layout()
    out = os.path.join(outdir, f"{name}.png")
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f"saved {out}")


def plot_group(mode, asmp, runs, outdir):
    """One 5x3 figure per (mode, assumption) overlaying every combo's error."""
    fig, axes = plt.subplots(5, 3, figsize=(14, 16), sharex=True)
    cmap = plt.cm.tab10
    any_data = False
    for ri, (label, csv) in enumerate(runs):
        try:
            df = pd.read_csv(csv)
        except Exception:
            continue
        any_data = True
        t = df["t"].to_numpy()
        col = cmap(ri % 10)
        for si, (state, (ec, sc, axl)) in enumerate(STATES.items()):
            for ai in range(3):
                ax = axes[si, ai]
                ax.plot(t, df[ec[ai]], color=col, lw=0.7,
                        label=label if (si == 0 and ai == 0) else None)
                s3 = df[sc[ai]].to_numpy()
                ax.plot(t, s3, color=col, lw=0.5, ls="--", alpha=0.35)
                ax.plot(t, -s3, color=col, lw=0.5, ls="--", alpha=0.35)
                if ri == 0:
                    ax.set_title(f"{state} {axl[ai]}", fontsize=9)
                    ax.grid(True, lw=0.3)
    if not any_data:
        plt.close(fig)
        return
    axes[0, 0].legend(fontsize=6, ncol=2, loc="upper left")
    for ax in axes[-1]:
        ax.set_xlabel("t [s]")
    fig.suptitle(f"{mode} / {asmp} — error +/- 3sigma (all combos)")
    fig.tight_layout()
    out = os.path.join(outdir, f"group_{slug(mode)}_{slug(asmp)}.png")
    fig.savefig(out, dpi=120)
    plt.close(fig)
    print(f"saved {out}")


def main():
    manifest, outdir = sys.argv[1], sys.argv[2]
    os.makedirs(outdir, exist_ok=True)
    rows = [l.rstrip("\n").split("\t") for l in open(manifest) if l.strip()]

    groups = OrderedDict()
    for mode, asmp, pose, bias, method, csv in rows:
        plot_case(mode, asmp, pose, bias, method, csv, outdir)
        groups.setdefault((mode, asmp), []).append(
            (f"{pose}-{bias.upper()}-{method}", csv))

    for (mode, asmp), runs in groups.items():
        plot_group(mode, asmp, runs, outdir)


if __name__ == "__main__":
    main()
