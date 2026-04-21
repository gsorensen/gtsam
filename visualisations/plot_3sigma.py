#!/usr/bin/env python3
"""Plot estimation errors with 3-sigma bounds in a 5x3 grid.

Compares up to four (preintegrator, bias) variants on the same axes:
  - SE3  + Constant Bias    → gtsam_fork_test_cb.csv
  - SE3  + Gauss-Markov     → gtsam_fork_test_gm.csv
  - SE23 + Constant Bias    → gtsam_fork_test_cb_se23.csv
  - SE23 + Gauss-Markov     → gtsam_fork_test_gm_se23.csv

Any subset that exists on disk will be plotted.
"""

import argparse
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ============================================================================
# Configuration
# ============================================================================

# (tag, filename_suffix, label, error_color, sigma_color,
#  err_linestyle, sig_dashes)
# Earlier entries are drawn last (on top). Reorder to change layering.
# err_linestyle distinguishes the 4 datasets when curves overlap (e.g. when
# aiding=none + init-from-truth makes SE3/SE23 visually identical).
# sig_dashes is a matplotlib dash tuple for the ±3σ lines so overlapping σ
# bounds stay individually visible.
PLOT_ORDER = [
    ("se23_gm", "gm_se23", "SE23 + GM", "tab:blue",   "tab:cyan",
     "solid",   (6, 2)),
    ("se23_cb", "cb_se23", "SE23 + CB", "tab:green",  "tab:olive",
     "dashed",  (4, 2)),
    ("se3_gm",  "gm",      "SE3 + GM",  "tab:red",    "tab:orange",
     "dashdot", (2, 2)),
    ("se3_cb",  "cb",      "SE3 + CB",  "tab:purple", "tab:pink",
     "dotted",  (1, 2)),
]

RESULTS_DIR = "/Users/ghms/ws/ntnu/parnav_ins_sim/results"

# ============================================================================


def tag_from_path(path: str) -> str:
    """Infer a PLOT_ORDER tag from an explicit CSV path."""
    base = os.path.basename(path)
    if "_se23" in base:
        return "se23_gm" if "_gm" in base else "se23_cb"
    return "se3_gm" if "_gm" in base else "se3_cb"


def main():
    parser = argparse.ArgumentParser(description="Plot 3-sigma error bounds")
    parser.add_argument(
        "--dir",
        default=RESULTS_DIR,
        help="Directory containing gtsam_fork_test_{cb,gm}[_se23].csv",
    )
    parser.add_argument(
        "--csv",
        nargs="*",
        help="Explicit CSV file path(s). Overrides --dir auto-detection.",
    )
    parser.add_argument(
        "--suffix",
        default="",
        help="Suffix appended before .csv when auto-detecting "
             "(e.g. '_none_10s'). Empty matches the default filenames.",
    )
    parser.add_argument(
        "--ylim",
        choices=("auto", "sigma", "error"),
        default="auto",
        help="Per-subplot y-limit strategy: "
             "'auto' = matplotlib default; "
             "'sigma' = max |3σ| across datasets, ±10%% padding "
             "(error lines may clip out when they diverge); "
             "'error' = max |error| across datasets, ±10%% padding.",
    )
    args = parser.parse_args()

    # Index PLOT_ORDER by tag for quick lookup.
    by_tag = {e[0]: e for e in PLOT_ORDER}

    # Build list of (label, dataframe, err_color, sig_color).
    # Draw back-to-front: last appended is drawn on top, matching PLOT_ORDER
    # where the first entry should appear in front.
    datasets = []

    if args.csv:
        for path in args.csv:
            entry = by_tag.get(tag_from_path(path), PLOT_ORDER[0])
            df = pd.read_csv(path)
            datasets.append(
                (entry[2], df, entry[3], entry[4], entry[5], entry[6]))
    else:
        for (tag, fname_suffix, label, err_col, sig_col, err_ls,
             sig_dashes) in reversed(PLOT_ORDER):
            path = os.path.join(
                args.dir,
                f"gtsam_fork_test_{fname_suffix}{args.suffix}.csv")
            if os.path.exists(path):
                df = pd.read_csv(path)
                datasets.append(
                    (label, df, err_col, sig_col, err_ls, sig_dashes))

    if not datasets:
        print(f"No result files found in {args.dir}")
        return

    rad2deg = np.degrees

    # Define the 5x3 grid: (error_col, sig3_col, label, unit, convert_to_deg)
    grid = [
        # Row 1: Position
        ("pos_err_n", "sig3_pos_n", "North position", "m", False),
        ("pos_err_e", "sig3_pos_e", "East position", "m", False),
        ("pos_err_d", "sig3_pos_d", "Down position", "m", False),
        # Row 2: Velocity
        ("vel_err_n", "sig3_vel_n", "North velocity", "m/s", False),
        ("vel_err_e", "sig3_vel_e", "East velocity", "m/s", False),
        ("vel_err_d", "sig3_vel_d", "Down velocity", "m/s", False),
        # Row 3: Attitude (rad -> deg)
        ("att_err_roll", "sig3_roll", "Roll", "deg", True),
        ("att_err_pitch", "sig3_pitch", "Pitch", "deg", True),
        ("att_err_yaw", "sig3_yaw", "Yaw", "deg", True),
        # Row 4: Accelerometer bias
        ("acc_bias_err_x", "sig3_ab_x", "Acc bias x", "m/s^2", False),
        ("acc_bias_err_y", "sig3_ab_y", "Acc bias y", "m/s^2", False),
        ("acc_bias_err_z", "sig3_ab_z", "Acc bias z", "m/s^2", False),
        # Row 5: Gyroscope bias (rad/s -> deg/s)
        ("gyro_bias_err_x", "sig3_gb_x", "Gyro bias x", "deg/s", True),
        ("gyro_bias_err_y", "sig3_gb_y", "Gyro bias y", "deg/s", True),
        ("gyro_bias_err_z", "sig3_gb_z", "Gyro bias z", "deg/s", True),
    ]

    n_datasets = len(datasets)
    title = " vs ".join(d[0] for d in datasets) if n_datasets > 1 else datasets[0][0]
    fig, axes = plt.subplots(5, 3, figsize=(16, 14), sharex=True)
    fig.suptitle(f"Estimation errors with 3-sigma bounds — {title}", fontsize=14)

    # Reduce sigma fill alpha when many datasets overlap.
    fill_alpha = 0.06 if n_datasets <= 2 else 0.04

    for i, (err_col, sig_col, label, unit, to_deg) in enumerate(grid):
        row, col = divmod(i, 3)
        ax = axes[row, col]

        max_sig = 0.0
        max_err = 0.0
        for ds in datasets:
            ds_label, df, color_err, color_sig, err_ls, sig_dashes = ds
            n = len(df)
            t = np.arange(n) * 0.1

            err = df[err_col].values
            sig3 = df[sig_col].values

            if to_deg:
                err = rad2deg(err)
                sig3 = rad2deg(sig3)

            max_sig = max(max_sig, float(np.nanmax(np.abs(sig3))))
            max_err = max(max_err, float(np.nanmax(np.abs(err))))

            suffix = f" ({ds_label})" if n_datasets > 1 else ""
            ax.plot(t, err, color=color_err, linewidth=0.9,
                    linestyle=err_ls, alpha=0.9,
                    label=f"Error{suffix}")
            ax.plot(t, sig3, color=color_sig, dashes=sig_dashes,
                    linewidth=0.9, alpha=0.9,
                    label=f"+3$\\sigma${suffix}")
            ax.plot(t, -sig3, color=color_sig, dashes=sig_dashes,
                    linewidth=0.9, alpha=0.9,
                    label=f"-3$\\sigma${suffix}")
            ax.fill_between(t, -sig3, sig3, color=color_sig, alpha=fill_alpha)

        # Per-subplot y-limits.
        if args.ylim in ("sigma", "error"):
            ref = max_sig if args.ylim == "sigma" else max_err
            if ref > 0 and np.isfinite(ref):
                pad = 1.1 * ref
                ax.set_ylim(-pad, pad)

        ax.set_ylabel(f"{label} [{unit}]", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.3)

    # Build a compact shared legend at the figure level (one entry per dataset
    # for the error line, plus a single dashed marker for ±3σ).
    from matplotlib.lines import Line2D
    handles = []
    for ds in datasets:
        ds_label, _, color_err, color_sig, err_ls, sig_dashes = ds
        handles.append(Line2D([0], [0], color=color_err, linewidth=1.4,
                              linestyle=err_ls,
                              label=f"Error ({ds_label})"))
        handles.append(Line2D([0], [0], color=color_sig, dashes=sig_dashes,
                              linewidth=1.4,
                              label=f"±3$\\sigma$ ({ds_label})"))
    fig.legend(handles=handles, loc="upper right", fontsize=7,
               ncol=min(n_datasets, 4), framealpha=0.9)

    for ax in axes[-1, :]:
        ax.set_xlabel("Time [s]", fontsize=9)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    out_name = os.path.join(args.dir, "3sigma_comparison.png")
    plt.savefig(out_name, dpi=150)
    print(f"Saved to {out_name}")
    plt.show()


if __name__ == "__main__":
    main()
