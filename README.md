# ME319 Assignment 2: Predictive Tracking and Turret Control for a Maneuvering Aerial Target

Guardian point-defense turret vs the V-44 drone (2-D elevation plane). The
team: Jithu (turret plant, feedback controller, analysis), Harsh (reference
shaping, firing logic, simulation harness), Sanjiban (target-state estimation
and prediction, paper). The problem statement is
`Assignment_2_ME_319_Control_Systems_S3.pdf`.

> ### Note for Claude (or any AI assistant used by a team member)
> - **Do not make crucial design decisions on your own.** Model choices, tuning values, interfaces between team members, and anything another member depends on are the user's (and the team's) call.
> - **Always ask before choosing a design.** Lay out the options, with their trade-offs and test evidence, in simple words, and let the user decide.
> - **Explain everything nicely.** Assume the user wants to understand why, not just get code.
> - **Don't silently change decisions already recorded** in this README, `params.m`, `TUNABLE_PARAMETERS.md`, `ESTIMATOR_MATH.md`, `TESTING_VALIDATION_METRICS.md` or the team's pipeline document. If something looks wrong, flag it to the user instead of fixing it yourself.

## 1. What the system does

```
radar (r, θ, 20 Hz, noisy, may black out)
  → estimator_step   Kalman filter in (x, y), Singer model    → θ̂, r̂, θ̂̇, σ_cross
  → predictor        θ_pred = θ̂ + t_p·θ̂̇, t_p = 0.5 s          (order n = 1)
  → reference_shaper prefilter F(z) with one zero (pole 0.52 s, zero 0.145 s), clamp −10°…85°
  → feedback_step    G_c = K_P·N(s): K_P = 8012 V/rad → ±600 V → notch N(z)
  → turret_plant     motor + 100:1 gear + turret, hard stops
  fire_logic (alongside): pulse when the track is valid, |d̂| < 2 m and P(hit) ≥ 0.95, ≥ 1.01 s since the last pulse
```

**Core idea.** The laser is instantaneous, so the turret must point at where
the target is *now*.
* The predictor adds a 0.5 s lead (θ̂ + 0.5 θ̂̇).
* The prefilter and feedback loop together add exactly 0.5 s of lag: F·T ≈ 1/(1 + 0.5 s).
* The two cancel, so the turret points at the current estimate θ̂(t).
* The fast inner loop (K_v = 8 1/s, about 0.125 s of lag) handles acquisition; the prefilter supplies the rest.
* The prefilter has one zero (D38), so the cancellation is exact up to s² on the real sampled loop. The earlier first-order prefilter left a 0.046·r·θ̈ pointing error (D33).
* The math of the estimator, predictor and firing rule is in `ESTIMATOR_MATH.md`; the data sets and metrics are in `TESTING_VALIDATION_METRICS.md`.
* All controller blocks are LTI and update on a 50 ms grid aligned with the radar.

Every design decision, with its reasoning, is in the team's pipeline document
(in the handover package: `02_pipeline/PIPELINE_full.md`).

## 2. Required functions (assignment section 3)

| Function | File | Notes |
|---|---|---|
| `[r_meas, theta_meas, meas_valid] = target_motion_generator(t, scenario)` | `target_motion_generator.m` | `scenario` = `'A'`, `'B'`, `'C'`, an integer id (random cases 4…; 1000+k = case k with a forced 0.5 s blackout) or a struct from `scenario_library.m`. White radar noise, one draw per 20 Hz sample; `meas_valid = false` between samples and in blackouts. |
| `[V_m, fire_cmd] = turret_controller(t, r_meas, theta_meas, meas_valid, theta_turret, theta_dot_turret)` | `turret_controller.m` | Calls `estimator_step` → `reference_shaper` → `feedback_step` → `fire_logic`. Safe when called from an ODE solver: repeated or slightly earlier times return the held outputs, and a backward jump of more than 0.5 s starts a new run. |
| `[theta_dot_turret, theta_ddot_turret, i_dot] = turret_plant(t, V_m, theta_turret, theta_dot_turret, i)` | `turret_plant.m` | First-principles electrical + mechanical model with rigid stops at −10° and +85°. The voltage limit is applied in the controller, not here. |

All constants are in `params.m` (the only place they are defined). The frozen
values are marked FROZEN there.

## 3. Requirements

* MATLAB (tested R2025a). Base MATLAB for everything except
  `design_controller.m` and `sensitivity_analysis.m`, which need the Control
  System Toolbox. The core code also runs in GNU Octave 8 (used for
  cross-checks), but the reported numbers and figures are from MATLAB.
* Python 3 with matplotlib, only to redraw the two diagrams (`docs/diagrams/make_diagrams.py`).

## 4. Reproducing every reported result

From the repository root in MATLAB:

```matlab
addpath(pwd); addpath('tools');
```

| Result | Command | Output | Time |
|---|---|---|---|
| Plant and loop checks (eigenvalues, open-loop step, hard stop, closed-loop step / slew / ramp) | `check_turret_plant` | Command Window (PASS/FAIL) + 4 figures | < 1 min |
| Controller design: margins, sampled margins, ±20 % robustness, Bode, root locus, step/ramp | `design_controller` | `figures/bode_loop.png`, `root_locus.png`, `step_ramp.png` | seconds |
| Sensitivity analysis A1-A4, B, and the closed-loop error budget A1b | `sensitivity_analysis` | `figures/sens_FT.png`, `sens_recovery.png`, `sens_acquisition.png` | < 1 min |
| Scenarios A-C, initial-angle sweep, 25 random cases, prefilter unit test | `run_all` | `figs/scenario_*.png`, `results/summary.csv`, `theta0_sweep.csv`, `random_cases.csv` | ~1 min |
| Figures A-C only | `for s = 'ABC', R = run_scenario(s); plot_scenario(R, fullfile('figs', ['scenario_' s '.png'])); end` | `figs/` | ~10 s |
| Train-G study: P_hit threshold sweep, reset on/off, threshold selection, train metrics (115 cases) | `compare_designs` | `results/compare_designs.md`, `results/train_metrics.md` + CSVs + logs | ~2 min |
| Reset ON vs OFF, case by case (needs `compare_designs` first) | `pair_reset` | Command Window | seconds |
| Solver-style call robustness | `test_call_robustness` | Command Window | ~1 min |
| Test and validation (46 + 160 runs) with metrics | `evaluate_tvt` | `results/TVT_report.md`, `results/test_metrics.md`, `results/val_metrics.md`, CSVs, `figures/tvt_tstat.png` | ~2 min |
| Estimator checks and tuning tables (Sanjiban) | `cd estimator_tests; check_estimator; run_estimator_sweeps` | `estimator_tests/results/` | ~10 min |
| Block diagram and drive schematic | `python3 docs/diagrams/make_diagrams.py` | `docs/diagrams/*.png/pdf/svg` | seconds |

The figures and result files in this repository were produced exactly this
way (MATLAB R2025a, 15 Nov 2026).

## 5. File map

| File | Owner | Role |
|---|---|---|
| `params.m` | Jithu (+ Harsh's T2 block) | All constants and design values; `params('override', s)` is used by the study scripts only. |
| `turret_plant.m`, `feedback_step.m` | Jithu | Plant model; feedback controller G_c = K_P·N with ±600 V and the D19 hold rule. |
| `design_controller.m`, `sensitivity_analysis.m` | Jithu | Controller-design and sensitivity figures and numbers. |
| `estimator_step.m`, `estimator_tests/`, `TUNABLE_PARAMETERS.md`, `docs/ESTIMATOR_README_sanjiban.md` | Sanjiban | Estimator and predictor (final), its tests and tuning record, his original README. |
| `turret_controller.m`, `reference_shaper.m`, `prefilter_design.m`, `fire_logic.m`, `turret_debug.m` | Harsh | Required wrapper, prefilter (design and run time), firing logic, logging side channel. |
| `target_motion_generator.m`, `target_scenario.m`, `target_truth.m`, `scenario_library.m` | Harsh (+ 15 Nov extensions) | Scenario generator (envelope-checked), truth accessor, designed test/validation scenarios. |
| `run_scenario.m`, `run_all.m`, `run_random.m`, `run_theta0_sweep.m`, `plot_scenario.m`, `print_metrics.m`, `write_csv.m`, `test_shaper.m` | Harsh | Closed-loop simulation harness (RK4 at 1 ms), metrics, figures. |
| `compare_designs.m`, `evaluate_tvt.m`, `metrics_report.m` | integration (15 Nov) | Train-G study and threshold selection; test and validation evaluation; the metrics of `TESTING_VALIDATION_METRICS.md`. |
| `ESTIMATOR_MATH.md`, `TESTING_VALIDATION_METRICS.md` | Sanjiban | Math of the estimator, predictor and firing rule; data sets and metrics. |
| `results/TVT_README.md`, `results/RERUN_2026-11-15.md` | integration | How to read the results; before/after summary of the D38–D41 re-run. |
| `tools/` | various | Verification scripts: `check_turret_plant`, `test_call_robustness`, `pair_reset`, `plant_rhs`. |
| `docs/tvt_parameters.md` | integration | Parameter list (given / set / derived / tuned) and data split; header of `results/TVT_report.md`. |
| `docs/diagrams/` | integration | Block diagram and drive schematic (script + PNG/PDF). |
| `figures/`, `figs/`, `results/` | generated | Paper figures and result tables (regenerate with section 4). |

## 6. Headline results (frozen design)

* **Scenarios A, B, C:** 9/9 hits each; first hit 1.20 / 1.25 / 1.50 s; tracking RMS 0.26 / 0.75 / 0.71 m. Peak drive demand is 600 V and 35 A during the first slew only.
* **Initial-angle sweep** (−10°…85°): 15/15 intercepted, 0 missed; worst first hit 2.55 s (slew from 85°).
* **Margins at the 50 ms update:** GM 11.0 dB, PM 61° (continuous 19.8 dB / 78°); the loop is unchanged.
* **Train-G** (115 cases, threshold fitted here): 115/115 intercepted, 1049/1049 pulses hit, first hit 0.86 s mean.
* **Test** (46 unseen cases): 46/46 intercepted, 416/416 pulses hit, first hit 1.04 s mean.
* **Validation** (16 scenarios × 10 seeds): 160/160 intercepted, 1438/1438 pulses hit, first hit 0.97 s mean / 1.65 s worst.

## 7. AI use

AI tools (Claude) were used throughout. See the AI appendix of the paper and
the notes in the handover package (`06_ai_appendix/`).
