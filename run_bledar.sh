#!/usr/bin/env bash
# Run run_bledar (multirotor BLE/DAR fusion) for one aiding source across the
# four (preint, bias) combinations, then plot with the shared plotter into the
# bledar_<aiding> figure/table folders.
#
#   SE3  + CB   -> bledar_cb.csv
#   SE3  + GM   -> bledar_gm.csv
#   SE23 + CB   -> bledar_cb_se23.csv
#   SE23 + GM   -> bledar_gm_se23.csv
#
# Usage:
#   ./run_bledar.sh [--aiding gnss|pars|uwb] [--base-path DIR]
#                   [--output-dir DIR] [--rmse group|axis|all|none]
#                   [--robust none|gm|tukey] [--fig-dir DIR] [--latex-dir DIR]
#                   [-- <extra binary args...>]
#
# Position error/RMSE is against the RTK truth (interpolated); velocity and
# attitude have no truth (those rows are zero). Figures go to
# <fig-dir>/bledar_<aiding>, tables to <latex-dir>/bledar_<aiding>.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="${BIN:-$REPO/_build/programs/run_bledar}"
PLOTTER="$REPO/visualisations/plot_3sigma.py"

AIDING="gnss"
BASE_PATH=""
OUTPUT_DIR=""
RMSE="group"
ROBUST=""
FIG_DIR=""
LATEX_DIR=""
EXTRA=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --aiding)     AIDING="$2"; shift 2 ;;
    --base-path)  BASE_PATH="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --rmse)       RMSE="$2"; shift 2 ;;
    --robust)     ROBUST="$2"; shift 2 ;;
    --fig-dir)    FIG_DIR="$2"; shift 2 ;;
    --latex-dir)  LATEX_DIR="$2"; shift 2 ;;
    --)           shift; EXTRA=("$@"); break ;;
    -h|--help)    grep '^#' "$0" | grep -v '^#!' | cut -c3-; exit 0 ;;
    *)            echo "Unknown argument: $1" >&2; exit 1 ;;
  esac
done

case "$AIDING" in gnss|pars|uwb) ;; *)
  echo "Invalid --aiding: $AIDING (expected gnss|pars|uwb)" >&2; exit 1 ;;
esac

if [[ ! -x "$BIN" ]]; then
  echo "Binary not found: $BIN" >&2
  echo "Build it first: cmake --build _build --target run_bledar" >&2
  exit 1
fi

BIN_ARGS=(--aiding "$AIDING")
[[ -n "$BASE_PATH" ]] && BIN_ARGS+=(--base-path "$BASE_PATH")
[[ -n "$OUTPUT_DIR" ]] && BIN_ARGS+=(--output-dir "$OUTPUT_DIR")
[[ -n "$ROBUST" ]] && BIN_ARGS+=(--robust "$ROBUST")

run() {
  local preint="$1" bias="$2"
  echo
  echo "=========================================================="
  echo "aiding=$AIDING  preint=$preint  bias=$bias"
  echo "=========================================================="
  "$BIN" --preint "$preint" --bias "$bias" \
    "${BIN_ARGS[@]}" ${EXTRA[@]+"${EXTRA[@]}"}
}

run se3  cb
run se3  gm
run se23 cb
run se23 gm

echo
echo "=== Plotting bledar_$AIDING ==="
VENV="${VENV:-$REPO/.venv}"
if [[ -f "$VENV/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source "$VENV/bin/activate"
fi
PLOT_ARGS=(--rmse "$RMSE" --result-prefix bledar_ --scenario "bledar_$AIDING")
[[ -n "$OUTPUT_DIR" ]] && PLOT_ARGS+=(--dir "$OUTPUT_DIR")
[[ -n "$FIG_DIR" ]] && PLOT_ARGS+=(--fig-dir "$FIG_DIR")
[[ -n "$LATEX_DIR" ]] && PLOT_ARGS+=(--latex-dir "$LATEX_DIR")
python "$PLOTTER" "${PLOT_ARGS[@]}"
