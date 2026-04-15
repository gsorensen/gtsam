#!/usr/bin/env python3
"""Plot estimation errors with 3-sigma bounds in a 5x3 grid.

Supports comparing Gauss-Markov and Constant Bias results side by side.
If both files exist, both are plotted on the same axes.
"""

import argparse
import os

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ============================================================================
# Configuration
# ============================================================================

# Plot order: the first entry is drawn on top (in front).
# Swap the two entries to change which model is plotted in front.
PLOT_ORDER = [
    ("gm", "Gauss-Markov", "blue", "red"),
    ("cb", "Constant Bias", "green", "orange"),
]

RESULTS_DIR = "/Users/ghms/ws/ntnu/parnav_ins_sim/results"

# ============================================================================


def main():
    parser = argparse.ArgumentParser(description="Plot 3-sigma error bounds")
    parser.add_argument(
        "--dir",
        default=RESULTS_DIR,
        help="Directory containing gtsam_fork_test_{gm,cb}.csv",
    )
    parser.add_argument(
        "--csv",
        nargs="*",
        help="Explicit CSV file path(s). Overrides --dir auto-detection.",
    )
    args = parser.parse_args()

    # Build list of (label, dataframe, err_color, sig_color)
    datasets = []

    if args.csv:
        # Explicit files provided
        for path in args.csv:
            tag = "gm" if "_gm" in os.path.basename(path) else "cb"
            entry = next((e for e in PLOT_ORDER if e[0] == tag), PLOT_ORDER[0])
            df = pd.read_csv(path)
            datasets.append((entry[1], df, entry[2], entry[3]))
    else:
        # Auto-detect from results directory, respecting PLOT_ORDER
        # Draw back-to-front: last in list is drawn last (on top)
        for tag, label, err_col, sig_col in reversed(PLOT_ORDER):
            path = os.path.join(args.dir, f"gtsam_fork_test_{tag}.csv")
            if os.path.exists(path):
                df = pd.read_csv(path)
                datasets.append((label, df, err_col, sig_col))

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

    for i, (err_col, sig_col, label, unit, to_deg) in enumerate(grid):
        row, col = divmod(i, 3)
        ax = axes[row, col]

        for ds_label, df, color_err, color_sig in datasets:
            n = len(df)
            t = np.arange(n) * 0.1

            err = df[err_col].values
            sig3 = df[sig_col].values

            if to_deg:
                err = rad2deg(err)
                sig3 = rad2deg(sig3)

            suffix = f" ({ds_label})" if n_datasets > 1 else ""
            ax.plot(t, err, color=color_err, linewidth=0.6,
                    label=f"Error{suffix}")
            ax.plot(t, sig3, color=color_sig, linestyle="--", linewidth=0.8,
                    label=f"+3$\\sigma${suffix}")
            ax.plot(t, -sig3, color=color_sig, linestyle="--", linewidth=0.8,
                    label=f"-3$\\sigma${suffix}")
            ax.fill_between(t, -sig3, sig3, color=color_sig, alpha=0.06)

        ax.set_ylabel(f"{label} [{unit}]", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.3)

        if row == 0 and col == 2:
            ax.legend(fontsize=6, loc="upper right", ncol=n_datasets)

    for ax in axes[-1, :]:
        ax.set_xlabel("Time [s]", fontsize=9)

    plt.tight_layout()
    out_name = os.path.join(args.dir, "3sigma_comparison.png")
    plt.savefig(out_name, dpi=150)
    print(f"Saved to {out_name}")
    plt.show()


if __name__ == "__main__":
    main()
