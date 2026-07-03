#!/usr/bin/env python3
"""Plot estimation errors with 3-sigma bounds in a 5x3 grid.

Compares up to four (preintegrator, bias) variants on the same axes:
  - SE3  + Constant Bias    -> gtsam_fork_test_cb.csv
  - SE3  + Gauss-Markov     -> gtsam_fork_test_gm.csv
  - SE23 + Constant Bias    -> gtsam_fork_test_cb_se23.csv
  - SE23 + Gauss-Markov     -> gtsam_fork_test_gm_se23.csv

Any subset that exists on disk will be plotted.

Modes:
  default        error line + +/-3 sigma envelope per dataset
  --mode sigma1  1-sigma (|sigma|) curve over time per dataset, no error line
"""

import argparse
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ============================================================================
# Configuration
# ============================================================================

# One color per (preint, bias) case. Error lines are drawn solid; sigma lines
# are drawn dashed — same dash pattern across all cases, colors distinguish.
#
# (tag, filename_suffix, label, color)
# Earlier entries draw last (on top).
PLOT_ORDER = [
    ("se23_gm", "gm_se23", "SE23 + GM", "tab:blue"),
    ("se23_cb", "cb_se23", "SE23 + CB", "tab:green"),
    ("se3_gm",  "gm",      "SE3 + GM",  "tab:red"),
    ("se3_cb",  "cb",      "SE3 + CB",  "tab:purple"),
]

# Shared line styles.
ERR_LINESTYLE = "solid"
SIG_DASHES = (4, 2)

RESULTS_DIR = "/Users/ghms/ws/ntnu/parnav_ins_sim/results"

# RMSE substate structure: (group_label, unit, convert_to_deg,
#                           [(component_label, error_col), ...]).
RMSE_STRUCTURE = [
    ("Position", "m", False,
     [("N", "pos_err_n"), ("E", "pos_err_e"), ("D", "pos_err_d")]),
    ("Velocity", "m/s", False,
     [("N", "vel_err_n"), ("E", "vel_err_e"), ("D", "vel_err_d")]),
    ("Attitude", "deg", True,
     [("roll", "att_err_roll"), ("pitch", "att_err_pitch"),
      ("yaw", "att_err_yaw")]),
    ("Acc bias", "m/s^2", False,
     [("x", "acc_bias_err_x"), ("y", "acc_bias_err_y"),
      ("z", "acc_bias_err_z")]),
    ("Gyro bias", "deg/s", True,
     [("x", "gyro_bias_err_x"), ("y", "gyro_bias_err_y"),
      ("z", "gyro_bias_err_z")]),
]


def _rmse_rows(detail):
    """Build the substate row list for a given detail level. Each row is
    (label, component_label, [error_cols], to_deg). There is no separate unit
    column: in 'all' mode the unit sits in `label` on the row below the group
    name; in 'group'/'axis' it is appended inline to the group name."""
    rows = []
    for grp, unit, to_deg, comps in RMSE_STRUCTURE:
        norm_cols = [c for _, c in comps]
        if detail == "group":
            rows.append((f"{grp} ({unit})", "", norm_cols, to_deg))
        elif detail == "axis":
            for clabel, col in comps:
                rows.append((f"{grp} ({unit})", clabel, [col], to_deg))
        else:  # all: norm row, then unit row + per-axis rows
            rows.append((grp, "Norm", norm_cols, to_deg))
            for i, (clabel, col) in enumerate(comps):
                rows.append((unit if i == 0 else "", clabel, [col], to_deg))
    return rows


# ============================================================================


def _substate_rmse(df, cols, to_deg):
    """RMSE of a substate. For multi-column groups this is the RMS of the
    per-sample vector magnitude: sqrt(mean(sum_axes err^2))."""
    sq = np.zeros(len(df))
    for c in cols:
        e = df[c].values
        if to_deg:
            e = np.degrees(e)
        sq = sq + e * e
    return float(np.sqrt(np.nanmean(sq)))


# Baseline and the four canonical (preint, bias) combinations. Each %impr
# column compares one variant against the SE3 + CB baseline.
RMSE_BASELINE = "SE3 + CB"
RMSE_COLS = ["SE3 + CB", "SE3 + GM", "SE23 + CB", "SE23 + GM"]
IMPR_COLS = [
    ("%impr of GM", "SE3 + GM"),
    ("%impr of SE23", "SE23 + CB"),
    ("%impr of SE23+GM", "SE23 + GM"),
]


def print_rmse_table(datasets, detail):
    """Print an RMSE table: one row per substate.

    Columns: the four RMSE values (SE3/SE23 x CB/GM), three %impr columns
    (each variant vs the SE3+CB baseline; green = better, red = worse), and
    a final column naming the lowest-RMSE combination for that substate.
    """
    substates = _rmse_rows(detail)

    use_color = sys.stdout.isatty()
    GREEN, RED, RESET = "\033[32m", "\033[31m", "\033[0m"

    def sign_color(cell, impr):
        if not use_color or impr == 0:
            return cell
        return f"{GREEN if impr > 0 else RED}{cell}{RESET}"

    by_label = {ds[0]: ds[1] for ds in datasets}

    grp_w = max([len("Substate")] + [len(s[0]) for s in substates]) + 2
    comp_w = max([0] + [len(s[1]) for s in substates])
    if comp_w:
        comp_w += 2
    col_w = max(12, max(len(c) for c in RMSE_COLS) + 2)
    impr_w = max(len(h) for h, _ in IMPR_COLS) + 2
    diff_header = "%diff SE3/SE23 GM"
    diff_w = len(diff_header) + 2

    header = "Substate".ljust(grp_w) + ("Comp".ljust(comp_w) if comp_w else "")
    header += "".join(c.rjust(col_w) for c in RMSE_COLS)
    header += "".join(h.rjust(impr_w) for h, _ in IMPR_COLS)
    header += diff_header.rjust(diff_w)
    header += "  " + "Best combination"
    print(f"\n=== RMSE ({detail}) ===")
    print(header)
    print("-" * len(header))

    for idx, (label, comp, cols, to_deg) in enumerate(substates):
        # Blank separator before each new group in 'all' mode.
        if detail == "all" and comp == "Norm" and idx:
            print()
        rmse = {lbl: _substate_rmse(by_label[lbl], cols, to_deg)
                for lbl in RMSE_COLS if lbl in by_label}
        row = label.ljust(grp_w) + (comp.ljust(comp_w) if comp_w else "")

        for lbl in RMSE_COLS:
            row += (f"{rmse[lbl]:>{col_w}.4f}" if lbl in rmse
                    else "n/a".rjust(col_w))

        base = rmse.get(RMSE_BASELINE)
        for _h, var in IMPR_COLS:
            if base and var in rmse:
                impr = (base - rmse[var]) / base * 100.0
                row += sign_color(f"{impr:>{impr_w}.2f}", impr)
            else:
                row += "n/a".rjust(impr_w)

        # %diff of SE23+GM relative to SE3+GM (isolates SE23 within GM).
        gm3, gm23 = rmse.get("SE3 + GM"), rmse.get("SE23 + GM")
        if gm3 and gm23 is not None:
            diff = (gm3 - gm23) / gm3 * 100.0
            row += sign_color(f"{diff:>{diff_w}.2f}", diff)
        else:
            row += "n/a".rjust(diff_w)

        best = min(rmse, key=rmse.get) if rmse else "n/a"
        row += "  " + best
        print(row)


def tag_from_path(path: str) -> str:
    """Infer a PLOT_ORDER tag from an explicit CSV path."""
    base = os.path.basename(path)
    if "_se23" in base:
        return "se23_gm" if "_gm" in base else "se23_cb"
    return "se3_gm" if "_gm" in base else "se3_cb"


def main():
    parser = argparse.ArgumentParser(description="Plot estimation errors / sigmas")
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
             "'sigma' = max |3 sigma| across datasets, +/-10%% padding "
             "(error lines may clip out when they diverge); "
             "'error' = max |error| across datasets, +/-10%% padding.",
    )
    parser.add_argument(
        "--mode",
        choices=("error3sigma", "sigma1"),
        default="error3sigma",
        help="'error3sigma' (default) plots error line with +/-3 sigma "
             "envelope; 'sigma1' plots the 1-sigma curve over time only.",
    )
    parser.add_argument(
        "--rmse",
        choices=("group", "axis", "all", "none"),
        default="group",
        help="Print an RMSE table: 'group' = 5 substates "
             "(position/velocity/attitude/acc bias/gyro bias magnitudes); "
             "'axis' = 15 per-axis substates (N/E/D, roll/pitch/yaw, ...); "
             "'all' = each group's norm followed by its per-axis rows; "
             "'none' = skip.",
    )
    args = parser.parse_args()

    # Index PLOT_ORDER by tag for quick lookup.
    by_tag = {e[0]: e for e in PLOT_ORDER}

    # Build list of (label, dataframe, color, err_ls, sig_dashes).
    # Draw back-to-front: last appended is drawn on top, matching PLOT_ORDER
    # where the first entry should appear in front.
    datasets = []

    if args.csv:
        for path in args.csv:
            entry = by_tag.get(tag_from_path(path), PLOT_ORDER[0])
            df = pd.read_csv(path)
            datasets.append((entry[2], df, entry[3]))
    else:
        for tag, fname_suffix, label, color in reversed(PLOT_ORDER):
            path = os.path.join(
                args.dir,
                f"gtsam_fork_test_{fname_suffix}{args.suffix}.csv")
            if os.path.exists(path):
                df = pd.read_csv(path)
                datasets.append((label, df, color))

    if not datasets:
        print(f"No result files found in {args.dir}")
        return

    if args.rmse != "none":
        print_rmse_table(datasets, args.rmse)

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

    mode_title = ("Estimation errors with 3-sigma bounds"
                  if args.mode == "error3sigma"
                  else "1-sigma over time")
    fig.suptitle(f"{mode_title} — {title}", fontsize=14)

    # Reduce sigma fill alpha when many datasets overlap.
    fill_alpha = 0.06 if n_datasets <= 2 else 0.04

    for i, (err_col, sig_col, label, unit, to_deg) in enumerate(grid):
        row, col = divmod(i, 3)
        ax = axes[row, col]

        max_sig = 0.0
        max_err = 0.0
        for ds in datasets:
            ds_label, df, color = ds
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

            if args.mode == "error3sigma":
                ax.plot(t, err, color=color, linewidth=0.9,
                        linestyle=ERR_LINESTYLE, alpha=0.9,
                        label=f"Error{suffix}")
                ax.plot(t, sig3, color=color, dashes=SIG_DASHES,
                        linewidth=0.9, alpha=0.9,
                        label=f"+3sigma{suffix}")
                ax.plot(t, -sig3, color=color, dashes=SIG_DASHES,
                        linewidth=0.9, alpha=0.9,
                        label=f"-3sigma{suffix}")
                ax.fill_between(t, -sig3, sig3, color=color, alpha=fill_alpha)
            else:  # sigma1
                sig1 = np.abs(sig3) / 3.0
                ax.plot(t, sig1, color=color, dashes=SIG_DASHES,
                        linewidth=1.0, alpha=0.95,
                        label=f"1sigma{suffix}")

        # Per-subplot y-limits.
        if args.mode == "error3sigma" and args.ylim in ("sigma", "error"):
            ref = max_sig if args.ylim == "sigma" else max_err
            if ref > 0 and np.isfinite(ref):
                pad = 1.1 * ref
                ax.set_ylim(-pad, pad)
        elif args.mode == "sigma1":
            # Sigma is non-negative; don't force symmetric limits.
            if args.ylim in ("sigma", "error") and max_sig > 0:
                ax.set_ylim(0.0, 1.1 * max_sig / 3.0)

        ax.set_ylabel(f"{label} [{unit}]", fontsize=8)
        ax.tick_params(labelsize=7)
        ax.grid(True, alpha=0.3)

    # Build a compact shared legend at the figure level.
    from matplotlib.lines import Line2D
    handles = []
    for ds in datasets:
        ds_label, _, color = ds
        if args.mode == "error3sigma":
            handles.append(Line2D([0], [0], color=color, linewidth=1.4,
                                  linestyle=ERR_LINESTYLE,
                                  label=f"Error ({ds_label})"))
            handles.append(Line2D([0], [0], color=color, dashes=SIG_DASHES,
                                  linewidth=1.4,
                                  label=f"+/-3sigma ({ds_label})"))
        else:
            handles.append(Line2D([0], [0], color=color, dashes=SIG_DASHES,
                                  linewidth=1.4,
                                  label=f"1sigma ({ds_label})"))
    fig.legend(handles=handles, loc="upper right", fontsize=7,
               ncol=min(n_datasets, 4), framealpha=0.9)

    for ax in axes[-1, :]:
        ax.set_xlabel("Time [s]", fontsize=9)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    out_basename = ("3sigma_comparison.png"
                    if args.mode == "error3sigma"
                    else "1sigma_comparison.png")
    out_name = os.path.join(args.dir, out_basename)
    plt.savefig(out_name, dpi=150)
    print(f"Saved to {out_name}")
    plt.show()


if __name__ == "__main__":
    main()
