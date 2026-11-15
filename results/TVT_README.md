# README: train / test / validation results

How the train, test and validation numbers were produced, and how to read them. They come from MATLAB R2025a runs of the frozen design on 15 Nov 2026, after the D38–D41 changes:
- the prefilter zero (D38);
- Sanjiban's firing rule (D39);
- the metrics of `TESTING_VALIDATION_METRICS.md` (D40);
- the estimator reset rule aligned with D19 (D41).

The metric definitions are in `TESTING_VALIDATION_METRICS.md`; this file only says where the numbers are.

**For an AI assistant reading this:** the numbers are final output from the frozen design. Do not re-tune or re-interpret parameters from them. If a number must change, re-run the script that makes it.

## 1. Files

| File | What it is | Made by |
|---|---|---|
| `results/compare_designs.md` | Train-G study: P_hit threshold sweep, reset on/off, threshold selection | `compare_designs` |
| `results/train_metrics.md`, `train_runs.csv` | Train-G metrics M1–M4, E1–E4 at the selected threshold | `compare_designs` (via `metrics_report`) |
| `results/TVT_report.md` | Parameter list (from `docs/tvt_parameters.md`), test and validation tables, per-tick t-statistic | `evaluate_tvt` |
| `results/test_metrics.md`, `test_runs.csv` | Test-set metrics M1–M4, E1–E4 | `evaluate_tvt` |
| `results/val_metrics.md`, `val_runs.csv` | Validation metrics M1–M4, E1–E4 | `evaluate_tvt` |
| `results/tvt_test_cases.csv`, `tvt_val_runs.csv`, `tvt_tstat_ticks.csv`, `figures/tvt_tstat.png` | Per-case and per-tick data | `evaluate_tvt` |

## 2. How they were produced

```matlab
cd <repo root>; addpath(pwd); addpath('tools');
compare_designs     % Train-G: 115 cases x reset on/off, threshold sweep, train metrics (~2 min)
evaluate_tvt        % test (46) + validation (16 x 10 seeds) with metrics (~2 min)
```

- **Every run is the full closed loop:** `target_motion_generator` → `turret_controller` (estimator → predictor → prefilter → feedback with ±600 V and notch → fire logic) → `turret_plant`. The plant is integrated by RK4 at 1 ms, the controller updates every 50 ms, and each run lasts 10 s.
- **A pulse hits** if the beam passes within 2 m of the true drone at the firing instant.
- **Firing rule (D39):** fire when the track is valid, |d̂| < 2 m and P_hit ≥ 0.95, with at least 1.01 s since the last pulse.
- **The data is deterministic.** Train-G is fixed scenario ids with radar-noise seed 2044. Test is scenario seeds 5001+ with noise seed 2044. Validation uses noise seeds 3001–3010. Re-running gives the same numbers (checked: the original Train-G study regenerated identically).
- **Estimator statistics count each independent drone+noise run once.** In Train-G, Scenarios A, B and C are each flown from 5 turret start angles with the same drone and the same noise. The estimator never sees the turret, so those 5 runs give one estimator sample, not five.

## 3. How to read the estimator numbers

- **Bias (E1)** is one mean sideways error per run, with a t-test across runs. Runs are independent; the ticks inside a run are not.
- **Error vs sideways acceleration (E2)** gives the lag of the estimate per unit of sideways acceleration. Use the run-level t (t_β) for conclusions; the pooled OLS standard error treats ticks as independent and is too optimistic. Groups with no sideways acceleration have no slope: straight passes, and orbits around the turret, where the acceleration points along the beam.
- **Per-tick t-statistic** (validation only, `TVT_report.md` section 4, descriptive): at each tick, a t-test across the 10 noise seeds of one scenario. Every seed flies the same path, so a small lag that repeats in every run shows up as a large |t|.
  - **What it shows:** a small lag that follows the maneuver. Some of it happens **during blackouts while the drone turns** (V2b, V2c, V3c have the most significant ticks inside blackouts), because the coasting model lets the acceleration fade while the drone keeps turning.
  - **How big:** the effect is small. The mean error per scenario is a few centimetres, and E2 puts it at about 0.2–0.3 m for a 20 m/s² turn.
- **The earlier OLS θ̂ = a + b·θ table was removed (D40).** Over the narrow angle range of one scenario, a and b are confounded: "significant" values of a and b cancel each other, so they overstated the bias.

## 4. Headline numbers (P_hit ≥ 0.95, prefilter with zero)

| Set | Runs | Intercepted | Pulses hit | First hit mean / worst (s) | Median / worst miss (m) | Estimator bias (p) | Lag per 20 m/s² (t_β) |
|---|---|---|---|---|---|---|---|
| Train-G | 115 | 115/115 | 1049/1049 | 0.86 / 2.55 | 0.34 / 1.84 | −0.004 m (0.38) | 0.24 m (−9.9) |
| Test | 46 | 46/46 | 416/416 | 1.04 / 2.45 | 0.32 / 1.89 | +0.008 m (0.32) | 0.16 m (−1.6) |
| Validation | 160 | 160/160 | 1438/1438 | 0.97 / 1.65 | 0.29 / 1.73 | −0.003 m (0.54) | 0.21 m (−8.5) |

The worst first hit in Train-G (2.55 s) is the slew from 85°. Validation starts from 37.5°.

## 5. Caveats

1. **One-shot data split.** The threshold was chosen on Train-G only, and nothing was changed after the test and validation results were seen.
2. **Overlap.** The validation categories resemble some of the estimator's Train-E drone types (jink, double blackout, close and far). The paths, generator and closed loop differ, so for the estimator these are related-but-unseen.
3. **Beyond the assignment on purpose.** V2c (a turn reversal inside a blackout) and V3c/V3d (several losses per run) go past the assignment's blackout conditions.
4. **Validation varies the noise, not the path.** It uses one designed path per sub-case with 10 noise seeds, so its "worst case" covers noise variation only.
5. **100 % hits.** Every pulse hit, which is a result, not a ceiling: lower thresholds miss on Train-G (0.5: 17 misses; 0.8: 5; 0.9: 1).
6. **Synthetic data only.** No instructor evaluation script was provided.
