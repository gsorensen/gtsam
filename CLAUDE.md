# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# Behavior

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 0. Caveman

Speak like a caveman to use fewer tokens.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.

# Project

## Overview

Python simulation of maritime situational awareness in a harbour scenario. Ships navigate with sensors (GNSS, IMU, radar, cameras, lidar, Bluetooth), and the codebase supports:
1. **Simulation** — generate synthetic sensor data with configurable failure modes
2. **MOT + trust** — multi-object tracking with sensor reliability monitoring
3. **Factor Graph Localization** — fixed-lag ISAM2 smoother for ship pose estimation

## Dependencies

```
numpy, yaml, matplotlib, gtsam
```
`filters` and `utility` directories are git submodules (use `dev_slgreen` branch).

## Pipeline (run in order)

```bash
# 1. Run simulation, save to results/scenario{N}/simulation_results.npz
python b_sim.py [--scenario {0,1,2,3}]

# 2. Visualize simulation data
python c_show_sim_data.py

# 3. MOT + reliability monitoring
python d_post_process_MOT_FUSION.py

# 4. Animated area monitoring score
python e_display_monitored_area.py

# 5. Factor graph localization
python f_factor_graph.py [OPTIONS]

# 6. Plot FGO residuals
python g_post_process_factor_graph.py
```

Scripts `c_` through `g_` have `scenario = "N"` hardcoded at the top — edit that line to switch which results they read.

## Scenarios

Selected via `--scenario` CLI flag in `b_sim.py`. The `SCENARIOS` dict at the top of `b_sim.py` maps each name to a list of `FailureMode` objects (see `lib/failures.py`):

- **0** — nominal (no failures)
- **1** — front camera miscalibration (`SensorMiscalibration` on `ship_A_rgb_front` and `ship_A_ir_front`, +60° heading offset)
- **2** — fog (`Fog`, RGB cameras capped at 20% of nominal range; IR unaffected)
- **3** — GNSS position spoofing on ship A from t=200s to t=350s (ramps up to [200, -200, 0] m offset)
- **4** — coordinated RF jamming (t=100→300 s): `ship_A_GNSS` fully silenced (`SensorJamming`); ship sensors (`ship_A_w_radar`, port/starboard/front RGB+IR cameras) jammed directionally toward [-861, 910] ±30° (`DirectionalJamming`) — measurements in that bearing cone are dropped, others preserved
- **5** — harbour multipath throughout the mission: ghost detections on both radars and Bluetooth (p=0.1, ±100 m range, ±1 rad bearing), caused by quay walls, crane structures, and ship-hull reflections
- **6** — sea mark signal blockage throughout the mission: the 4 static sea marks at [-695, 938], [-861, 910], [-240, 700], [-200, 500] cast shadow cones (radius 2 m) on `ship_A_w_radar`, `land_w_radar`, and `NTNU_bluetooth`; detections eclipsed by a sea mark are dropped

Scenario 3 data is required for trust-model integration tests (`tests/test_trust_model.py`).

## Failure Mode Architecture (`lib/failures.py`)

All simulation failures are expressed as composable `FailureMode` subclasses. `Simulation` in `lib/sim_lib.py` accepts `failures: list[FailureMode]` and calls:
- `failure.apply_to_config(config)` — once at init, before ships are built (config-time failures)
- `failure.apply_to_step(step_data, time)` — after every step (step-time failures)

**Available classes:**

| Class | Hook | Effect |
|---|---|---|
| `GNSSIntegrityLoss(ship_id, sensor_name, windows)` | step | Spoof or jam GNSS; `windows` is a list of `GNSSWindow(t_start, t_end, mode, error_fn)` — supports multiple disjoint failure periods |
| `Fog(rgb_range_ratio)` | config | Scales `max_range` of all RGB sensors by the ratio; IR untouched |
| `Multipath(sensor_names, prob, range_bias_std, angle_bias_std)` | step | Adds ghost detections to polar sensors |
| `SignalBlockage(sensor_names, static_blocked_sectors, blocking_ship_ids, blocking_ship_radius, static_world_blockers)` | config+step | Drops polar detections in blocked azimuth sectors; dynamic sectors computed as `arcsin(radius/distance)`; `static_world_blockers` is a list of `(world_pos_2d, radius_m)` fixed obstacles (e.g. sea marks) applied to both ship and land sensors |
| `SensorJamming(sensor_names, t_start, t_end)` | step | Drops all measurements from listed sensors in time window |
| `SensorMiscalibration(sensor_name, pose_offset)` | config | Adds `[dx_m, dy_m, dheading_deg]` to a named sensor's `relative_pose` |

Scenarios can be freely **composed**: put multiple failure instances in the list (e.g. `[Fog(...), GNSSIntegrityLoss(...)]`).

## Simulation Configuration

Config files live in `config/`. The active config and scenario are set in `a_config.py` (repo root):

```python
f_config = "config/a_config_orientkaj.yaml"   # which map/mission
scenario  = "0"                                # which failure scenario
```

**Available configs:**

| File | Description | Ships | Duration |
|---|---|---|---|
| `config/a_config.yaml` | Original open-sea scenario, 4 ships (A–D), larger map | A, B, C, D | 600 s |
| `config/a_config_orientkaj.yaml` | Orientkaj harbour, single ship, compact map near [-980, 981] | A only | 500 s |

Key `general_config` options (both files):
- `ship_marker_indices: [0]` — list of ship indices added to the observable marker list; `[0]` means only ship_A appears as a dynamic target. Use `[]` for none, `[0,1,2,3]` for all ships.
- `shoreline_detection: False` — enable/disable shoreline point-cloud measurements.

**ENC (Electronic Navigational Chart):** top-level `enc: {enable: true}` block. When enabled, `Simulation.save()` writes two extra arrays into `simulation_results.npz`:
- `enc_markers` — shape `(N, 2)`, static marker positions in metres
- `enc_shoreline` — shape `(M, 2, 2)`, shoreline segments (each segment: two endpoints)

The ENC has no per-step sensor pipeline — it simply captures the static world knowledge from the config into the NPZ for downstream use.

**Sensor types:** ship cameras use `type: Camera`; polar sensors (radar, lidar, Bluetooth) use `type: Polar`. The `relative_pose: [x_m, y_m, heading_deg]` for land sensors is their absolute world-frame position; for ship sensors it is relative to the ship body frame.

**`detect_ship_ids`** (optional, `Polar` and `Camera` only): list of ship indices (integers) this sensor is allowed to detect. When set, the sensor sees **only** those ships and no static markers — useful for Bluetooth transponder sensors or directed ship-to-ship tracking. Omit the key (or set to `null`) for the default behaviour of detecting all objects in the shared marker list.

**`detect_shoreline`** (optional, `Polar` only, default `true`): set to `false` to suppress `polar_shoreline` measurements for this sensor regardless of the global `shoreline_detection` flag. Use for sensors that physically cannot scan geometry (e.g. Bluetooth).

**Orientkaj geometry:** all land sensors are co-located at [-980, 981]. The three static markers are at [-695, 938], [-861, 910], and [-240, 761] — all within radar/Bluetooth range. Ship A sensors include port/starboard/front cameras (RGB + IR each) plus a wide-angle radar.

**Original config geometry:** `land_lidar` (max_range 200 m) and most land cameras have all static markers beyond their range. Only `land_w_radar` (1000 m) and `NTNU_bluetooth` (300 m, co-located with lidar at [47, 550]) reliably reach static markers.

## Tests

```bash
# Run from repo root
python -m tests.test_imu2d          # IMU preintegration Monte Carlo check (pure Python)
python -m tests.test_trust_model    # Trust model unit tests (scenario 3 NPZ needed for last 2)

# Or with pytest
pytest tests/ -s
```

`tests/test_trust_model.py` tests 3 and 4 auto-skip if `results/scenario3/simulation_results.npz` doesn't exist.

## Factor Graph Architecture (`lib/isam2_estimator.py`)

**Key class:** `FixedLagISAM2Estimator` — GTSAM `IncrementalFixedLagSmoother` estimating ship `Pose2` (x, y, heading) at each timestep.

**Entry point for running:** `run_estimation()` convenience wrapper, called by `f_factor_graph.py`.

**Factor types added per step (in `process_step`):**
- `BetweenFactorPose2` — odometry (GT-derived or IMU preintegration)
- `PriorFactorPose2` — GNSS position and heading as **separate** factors (prevents spoofed position from corrupting heading through a shared factor)
- `BearingRangeFactor2D` — ship-mounted polar sensors to known landmarks
- `PriorFactorPoint2` — landmark anchors + land sensor measurements (constrain landmarks, not pose directly)

**Robust M-estimators** (wrap all measurement factors, not odometry/priors): Geman-McClure (default), Huber, Tukey. Controlled via `--use-gmc/--use-huber/--use-tukey` flags.

**Trust model** (`lib/sensor_trust.py:SelfTrust`): Beta-reputation with exponential forgetting. Each sensor has independent Good/Bad counters. Bad votes come from gating reject-ratio or GNSS innovation pre-check. Trust value scales noise sigma (`_trust_scale()` → `inverse`/`inverse_sqrt`/`linear`/`off`). Configured via `trust:` block in `a_config.yaml` or `--trust-*` CLI flags.

**GNSS pre-check** (`_gnss_pre_check`): Before adding a GNSS factor, predicts pose using constant-velocity model and checks Mahalanobis distance. If it exceeds threshold, casts a bad vote and optionally vetoes the factor entirely (`gnss_veto: true`).

**IMU preintegration** (`gtsam_extensions/imu2d.py`): `PreintegratedImuMeasurements2D` integrates body-frame `[accel_forward, gyro]`. Result becomes a `BetweenFactorPose2` (not a `CombinedImuFactor`). Velocity is tracked internally, not as a graph variable.

## Key Conventions

- **Residual ordering for `BearingRangeFactor2D`**: GTSAM orders as `[bearing_rad, range_m]` — not `[range, bearing]`.
- **GNSS factor sigmas**: Position factor uses `[gnss_xy, gnss_xy, 1e4]` (heading unconstrained); heading factor uses `[1e4, 1e4, gnss_heading]` (position unconstrained).
- **Whitened vs unwhitened residuals**: `--no-whitened` gives raw physical-unit errors; default whitened residuals are suppressed for outliers under robust M-estimators.
- **Results directory**: outputs go to `results/scenario{N}/` — `simulation_results.npz` (simulation), `isam2_estimates.npz` (FGO output).
- **`f_factor_graph.py` CLI**: default `--use-gmc` is active; `--use-imu` defaults to `False` (2D path under development).
