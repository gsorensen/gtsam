#!/usr/bin/env bash
# Run SimulationFixedLagSmoother for one aiding scheme across all four
# (preint, bias) variants, then plot the 3-sigma error comparison.
#
#   SE3  + CB   -> gtsam_fork_test_cb.csv
#   SE3  + GM   -> gtsam_fork_test_gm.csv
#   SE23 + CB   -> gtsam_fork_test_cb_se23.csv
#   SE23 + GM   -> gtsam_fork_test_gm_se23.csv
#
# Usage:
#   ./run_sim.sh [--aiding gnss|pars|none] [--output-dir DIR]
#                [--rmse group|axis|all|none] [-- <extra sim args...>]
#
# --rmse selects the RMSE table granularity in the plot output:
#   group (default) = 5 substates, axis = 15 per-axis,
#   all = each group's norm + its per-axis rows, none = skip.
#
# Extra args after `--` are forwarded verbatim to every sim run
# (e.g. -- --duration 120 --input /path/to.csv).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="${BIN:-$REPO/_build/programs/SimulationFixedLagSmoother}"
PLOTTER="$REPO/visualisations/plot_3sigma.py"

AIDING="pars"
OUTPUT_DIR=""          # empty -> let the binary use its default dir
RMSE="group"           # RMSE table granularity: group|axis|none
EXTRA=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --aiding)     AIDING="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --rmse)       RMSE="$2"; shift 2 ;;
    --)           shift; EXTRA=("$@"); break ;;
    -h|--help)    grep '^#' "$0" | grep -v '^#!' | cut -c3-; exit 0 ;;
    *)            echo "Unknown argument: $1" >&2; exit 1 ;;
  esac
done

case "$AIDING" in gnss|pars|none) ;; *)
  echo "Invalid --aiding: $AIDING (expected gnss|pars|none)" >&2; exit 1 ;;
esac

if [[ ! -x "$BIN" ]]; then
  echo "Binary not found: $BIN" >&2
  echo "Build it first: cmake --build _build --target SimulationFixedLagSmoother" >&2
  exit 1
fi

DIR_ARGS=()
[[ -n "$OUTPUT_DIR" ]] && DIR_ARGS=(--output-dir "$OUTPUT_DIR")

run() {
  local preint="$1" bias="$2"
  echo
  echo "=========================================================="
  echo "aiding=$AIDING  preint=$preint  bias=$bias"
  echo "=========================================================="
  "$BIN" --aiding "$AIDING" --preint "$preint" --bias "$bias" \
    ${DIR_ARGS[@]+"${DIR_ARGS[@]}"} ${EXTRA[@]+"${EXTRA[@]}"}
}

run legacy cb
run legacy gm
run se23   cb
run se23   gm

echo
echo "=== Plotting 3-sigma error comparison ==="
VENV="${VENV:-$REPO/.venv}"
if [[ -f "$VENV/bin/activate" ]]; then
  # shellcheck disable=SC1091
  source "$VENV/bin/activate"
fi
PLOT_ARGS=(--rmse "$RMSE")
[[ -n "$OUTPUT_DIR" ]] && PLOT_ARGS+=(--dir "$OUTPUT_DIR")
python "$PLOTTER" "${PLOT_ARGS[@]}"
