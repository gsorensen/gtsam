#!/usr/bin/env python3
"""Plot estimation errors with 3-sigma bounds in a 5x3 grid."""

import argparse
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def main():
    parser = argparse.ArgumentParser(description="Plot 3-sigma error bounds")
    parser.add_argument(
        "csv",
        nargs="?",
        default="/Users/ghms/ws/ntnu/parnav_ins_sim/results/gtsam_fork_test.csv",
        help="Path to the results CSV file",
    )
    args = parser.parse_args()

    df = pd.read_csv(args.csv)
    n = len(df)
    t = np.arange(n) * 0.1  # 10 Hz update rate

    # Define the 5x3 grid: (error_col, sig3_col, label, unit)
    grid = [
        # Row 1: Position
        ("pos_err_n", "sig3_pos_n", "North position", "m"),
        ("pos_err_e", "sig3_pos_e", "East position", "m"),
        ("pos_err_d", "sig3_pos_d", "Down position", "m"),
        # Row 2: Velocity
        ("vel_err_n", "sig3_vel_n", "North velocity", "m/s"),
        ("vel_err_e", "sig3_vel_e", "East velocity", "m/s"),
        ("vel_err_d", "sig3_vel_d", "Down velocity", "m/s"),
        # Row 3: Attitude
        ("att_err_roll", "sig3_roll", "Roll", "rad"),
        ("att_err_pitch", "sig3_pitch", "Pitch", "rad"),
        ("att_err_yaw", "sig3_yaw", "Yaw", "rad"),
        # Row 4: Accelerometer bias
        ("acc_bias_err_x", "sig3_ab_x", "Acc bias x", "m/s^2"),
        ("acc_bias_err_y", "sig3_ab_y", "Acc bias y", "m/s^2"),
        ("acc_bias_err_z", "sig3_ab_z", "Acc bias z", "m/s^2"),
        # Row 5: Gyroscope bias
        ("gyro_bias_err_x", "sig3_gb_x", "Gyro bias x", "rad/s"),
        ("gyro_bias_err_y", "sig3_gb_y", "Gyro bias y", "rad/s"),
        ("gyro_bias_err_z", "sig3_gb_z", "Gyro bias z", "rad/s"),
    ]

    fig, axes = plt.subplots(5, 3, figsize=(16, 14), sharex=True)
    fig.suptitle("Estimation errors with 3-sigma bounds", fontsize=14)

    for i, (err_col, sig_col, label, unit) in enumerate(grid):
        row, col = divmod(i, 3)
        ax = axes[row, col]

        err = df[err_col].values
        sig3 = df[sig_col].values

        ax.plot(t, err, "b-", linewidth=0.6, label="Error")
        ax.plot(t, sig3, "r--", linewidth=0.8, label="+3$\\sigma$")
        ax.plot(t, -sig3, "r--", linewidth=0.8, label="-3$\\sigma$")
        ax.fill_between(t, -sig3, sig3, color="red", alpha=0.08)

        ax.set_ylabel(f"{label} [{unit}]", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.3)

        if row == 0 and col == 2:
            ax.legend(fontsize=7, loc="upper right")

    for ax in axes[-1, :]:
        ax.set_xlabel("Time [s]", fontsize=9)

    plt.tight_layout()
    plt.savefig(args.csv.replace(".csv", "_3sigma.png"), dpi=150)
    plt.show()


if __name__ == "__main__":
    main()
