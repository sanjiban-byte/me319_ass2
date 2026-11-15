# Predictive Tracking and Lag-Matched Turret Control

ME319 Control Systems, Assignment 2 (IIT Bombay, Autumn 2026). MATLAB code for a laser turret that tracks and fires at a maneuvering V-44 drone, using noisy 20 Hz radar readings that can black out for up to 0.5 s.

## What it does

```
radar (r, θ, 20 Hz, noisy, may black out)
  → estimator_step     Kalman filter in (x, y), Singer acceleration model → θ̂, r̂, θ̂̇, σ_cross
  → predictor          θ_pred = θ̂ + t_p·θ̂̇, t_p = 0.5 s
  → reference_shaper   prefilter F(z) with one pole and one zero, clamp to -10°…85°
  → feedback_step      G_c = K_P·N(s): gain 8012 V/rad → ±600 V → notch on the 56 rad/s mode
  → turret_plant       DC motor + 100:1 gearbox + turret, hard stops at -10° and +85°
  fire_logic           pulse when the track is valid, |d̂| < 2 m and P_hit ≥ 0.95, at least 1.01 s apart
```

The predictor adds 0.5 s of lead. The prefilter and feedback loop together add 0.5 s of lag, so the two cancel and the turret points at the current estimate of the drone when the (instantaneous) laser fires. All controller blocks update on a 50 ms grid aligned with the radar. The main design choices and their evidence are in [`DESIGN_DECISIONS.md`](DESIGN_DECISIONS.md).

## Requirements

- MATLAB (tested on R2025a). The simulation, controller and evaluation code use base MATLAB only.
- Control System Toolbox, only for `design_controller.m` and `sensitivity_analysis.m`.
- Python 3 with matplotlib, only to redraw the two diagrams.

## Quick start

From the repository root in MATLAB:

```matlab
addpath(pwd); addpath('tools');
R = run_scenario('B');      % Scenario A, B or C; 10 s closed-loop run
print_metrics(R);           % hits, first-hit time, tracking error, peak V and I
plot_scenario(R);           % range, elevation, pointing error with pulses, voltage, current
```

## Required functions (assignment section 3)

| Function | File |
|---|---|
| `[r_meas, theta_meas, meas_valid] = target_motion_generator(t, scenario)` | `target_motion_generator.m` (scenario `'A'`, `'B'`, `'C'`, an integer id for random cases, or a struct from `scenario_library.m`) |
| `[V_m, fire_cmd] = turret_controller(t, r_meas, theta_meas, meas_valid, theta_turret, theta_dot_turret)` | `turret_controller.m` |
| `[theta_dot_turret, theta_ddot_turret, i_dot] = turret_plant(t, V_m, theta_turret, theta_dot_turret, i)` | `turret_plant.m` |

All constants and design values are in `params.m`. **Before running `turret_controller` from your own evaluation script, read [`NOTE.md`](NOTE.md)**: it lists how the controller expects to be called and what it assumes.

## Reproducing the results in the paper

Run from the repository root after `addpath(pwd); addpath('tools');`. Every script below writes its own outputs; the committed figures and tables were produced exactly this way.

| Paper item | Command | Output | Time |
|---|---|---|---|
| Fig. 1 (schematic), Fig. 2 (block diagram) | `cd docs/diagrams; python make_diagrams.py` (in a shell) | `docs/diagrams/*.png/pdf/svg` | seconds |
| Table I (margins), Fig. 3 (root loci) | `design_controller` | Command Window, `figures/root_locus.png`, `bode_loop.png`, `step_ramp.png` | seconds |
| Fig. 4, Sec. IV-C to IV-E numbers (noise gains, recovery step, acquisition), error budget in Table III | `sensitivity_analysis` | Command Window, `figures/sens_*.png` | < 1 min |
| Table II (estimator selection) | `cd estimator_tests; run_estimator_sweeps; cd ..` | `estimator_tests/results/*.md` | ~10 min |
| Table III, Fig. 5, initial-angle sweep, random cases | `run_all` | `figs/scenario_*.png`, `results/summary.csv`, `theta0_sweep.csv`, `random_cases.csv` | ~1 min |
| Threshold sweep and selection, prefilter restart study, Table V (train) | `compare_designs` | `results/compare_designs.md`, `results/train_metrics.md`, CSVs | ~2 min |
| Sec. IV-E restart ON vs OFF, case by case (after `compare_designs`) | `pair_reset` | Command Window | seconds |
| Table V (test, validation), Fig. 6 | `evaluate_tvt` | `results/TVT_report.md`, `results/test_metrics.md`, `results/val_metrics.md`, `figures/tvt_tstat.png` | ~2 min |
| Table IV, first-order column | Set `p.prefilter_zero = false` in `params.m`, run `clear params`, then `run_all` and `sensitivity_analysis`; set it back to `true` afterwards | as above | ~2 min |

Correctness checks:

| Check | Command |
|---|---|
| Estimator (7 checks, PASS/FAIL) | `cd estimator_tests; check_estimator; cd ..` |
| Prefilter design (lag cancellation up to s², time-domain test) | `test_shaper` |
| Same pulses under solver-style calls (repeats, small steps back) | `test_call_robustness` |
| Plant model (eigenvalues, open-loop step, hard stop) | `check_turret_plant` |

A guide to every file in `results/`, the before/after numbers of the final revision, and caveats is in [`results/RERUN_2026-11-15.md`](results/RERUN_2026-11-15.md).

## Data: how train, test and validation sets are generated

All trajectories are checked against the assignment envelope (800-2000 m, ≤ 150 m/s, ≤ 20 m/s², elevation -10° to +85°, blackouts ≤ 0.5 s from t ≥ 1 s), and all radar noise is seeded white Gaussian noise (σ_r = 1 m, σ_θ = 0.02°). Re-running a script regenerates the same data exactly.

| Set | Used for | Generated by | Contents and seeds |
|---|---|---|---|
| Train-E | Estimator model and parameters | `estimator_tests/make_test_drones.m`, `run_estimator_sweeps.m` | 12 drone types; seed 1 with 30 runs per type (sweeps), seed 2 with 40 runs per type (confirmation) |
| Train-G | Firing threshold, prefilter restart after a blackout | `target_scenario.m`, `compare_designs.m` | 115 closed-loop cases: Scenarios A, B, C from turret angles -10°, 10°, 37.5°, 60°, 85°; random cases 4-53; the same with a forced 0.5 s blackout (ids 1004-1053); noise seed 2044 |
| Test | Checking the tuned values on new drones | `scenario_library('test')`, `evaluate_tvt.m` | 46 cases in three new families (multi-jink, two-frequency weave, sustained turn); trajectory seeds 5001+, noise seed 2044, random start angles |
| Validation | Final evaluation | `scenario_library('val')`, `evaluate_tvt.m` | 16 designed scenarios in 4 categories (near range/speed limits, full-a_max turns, radar losses, very close/far) × noise seeds 3001-3010 |

Metric definitions are in [`TESTING_VALIDATION_METRICS.md`](TESTING_VALIDATION_METRICS.md).

## Repository layout

| Path | Contents |
|---|---|
| `params.m` | All constants and design values |
| `target_motion_generator.m`, `target_scenario.m`, `target_truth.m`, `scenario_library.m` | Scenario generator, truth trajectories, designed test and validation scenarios |
| `turret_controller.m` | Required controller: calls `estimator_step` → `reference_shaper` → `feedback_step`, with `fire_logic` alongside |
| `estimator_step.m` | Kalman filter (Singer model) and predictor |
| `reference_shaper.m`, `prefilter_design.m` | Prefilter design (matched to the sampled loop) and run-time filter with clamp |
| `feedback_step.m`, `turret_plant.m` | Feedback compensator with voltage limit; plant model with hard stops |
| `fire_logic.m` | Firing gate and one-update pulses |
| `run_scenario.m`, `run_all.m`, `run_random.m`, `run_theta0_sweep.m`, `plot_scenario.m`, `print_metrics.m`, `write_csv.m`, `turret_debug.m` | Simulation harness (RK4 at 1 ms), metrics, figures, logging |
| `design_controller.m`, `sensitivity_analysis.m` | Controller design figures and sensitivity numbers |
| `compare_designs.m`, `evaluate_tvt.m`, `metrics_report.m` | Train-G study and threshold selection; test and validation evaluation; metrics |
| `test_shaper.m`, `tools/` | Verification scripts |
| `estimator_tests/` | Estimator data generator, sweeps, checks and result tables |
| `docs/tvt_parameters.md` | Every parameter, labelled given / set / derived / tuned, with its train data |
| `docs/diagrams/` | Schematic and block diagram (script and images) |
| `figures/`, `figs/`, `results/` | Generated figures and result tables |
| `DESIGN_DECISIONS.md`, `NOTE.md`, `ESTIMATOR_MATH.md`, `TESTING_VALIDATION_METRICS.md`, `TUNABLE_PARAMETERS.md` | Design decisions; notes for evaluation scripts; estimator, predictor and firing math; data sets and metrics; estimator tuning record |

Tags such as D19 or D38 in code comments and documents are labels from the team's decision log; the main decisions are summarised in `DESIGN_DECISIONS.md`.
