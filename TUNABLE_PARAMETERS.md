# Tunable Parameters: Estimation, Prediction and Firing Gate

Every value here is **tuned once** over many simulated drones and blackouts, then frozen. The assignment forbids retuning for individual scenarios or evaluation cases.

Fixed problem constants (radar noise, t_p = 0.5 s, limits) are **not** listed here. They already live in `params.m`.

**No hard-set values for sweepable parameters.** Until a value is Frozen, code must read it through an override (for example `estimator_step('config', overrides)`), not only from `params.m`. `params()` caches its result in a `persistent` variable, so a sweep cannot change it between runs without `clear params`.

Status key: **Open** = not decided yet · **Candidate** = a value is being tested · **Frozen** = final, copied into `params.m`

Decision references (D…, Q…) point to Jithu's work-split document, rev 3.

## Estimator (denoiser) — Sanjiban

| Parameter | Proposed name in `params.m` | Meaning | Starting value or range | Status |
|---|---|---|---|---|
| Model | `est_model` | `'CA'` (pure constant acceleration) or `'singer'` (acceleration that fades) | **`'singer'`** (chosen by Sanjiban 2026-11-09 from the MATLAB sweep; see results below) | Candidate |
| CA process noise | `est_q_ca` | Strength of the unknown jerk (CA only), as a continuous-time intensity, so it works at any call rate | **250** (best CA in the sweep; tested 100–20000 m²/s⁵). Not used while the model is Singer. | Candidate (backup) |
| Singer acceleration size | `est_sigma_m` | Typical size of the drone's acceleration, per axis (Singer only) | **10 m/s²** (tested 5–20) | Candidate |
| Singer fade time | `est_tau_a` | How long an acceleration typically lasts before fading (Singer only) | **1 s** (tested 0.25–2) | Candidate |
| Start-up speed uncertainty | `est_sigma_v0` | How unsure we are of speed at the first reading | 150 m/s (max speed) | Candidate |
| Start-up acceleration uncertainty | `est_sigma_a0` | How unsure we are of acceleration at the first reading | 20 m/s² (max accel) | Candidate |
| Readings before ready | `est_min_updates` | `est.valid` cannot turn true before this many radar readings have been used | 5 (0.25 s) | Candidate |
| Ready threshold | `est_valid_sigma` | …and the sideways uncertainty σ_cross must also be below this. Once true, `est.valid` stays true until a reset. Harsh's prefilter start (it uses θ̂̇) and the fire gate wait for it. | 1 m | Candidate |
| Coasting flag delay | `est_coast_gap` | `est.coasting` turns true when no reading has arrived for this long | 0.075 s (1.5 radar intervals) | Candidate |

**Fixed behaviour (not tunable):**
- **A reading is used** only if `meas_valid = 1` **and** (`r_meas`, `theta_meas`) differ from the last reading used. This works whether the caller sends `meas_valid = 1` only at the 20 Hz instants or keeps repeating the last reading. Real noisy readings never repeat exactly. (Replaces the earlier "≥ 40 ms gap" idea: if the caller repeats readings, a gap rule would sometimes take a stale repeat as new and then reject the real next one.)
- **Before the first reading** every output is NaN and `est.valid = false`. After it, outputs are real numbers at once (so the turret can start moving toward the drone if Harsh wants), but `est.valid` stays false until the track settles.

### Selection rule: optimise the worst case
We choose the model and its values by the **worst-case** chance that a shot hits, over every drone type in the test set, not by the average. A single leak over Mumbai matters more than a good average.

- Pure constant acceleration (CA) is chosen if its average is not much worse than the best-tuned Singer (within the run-to-run noise of the test) **and** its worst case is better.
- Otherwise the best-tuned Singer is chosen.
- The decision is made only from the MATLAB test set, never from rough Python numbers.

Because the turret effectively points at our **current** estimate θ̂(t) (pairing rule D2, see below), this comparison mainly measures how well θ̂(t) follows a maneuvering drone and how well it coasts through a blackout. It can therefore be tested on the estimator alone: sideways miss ≈ r · |θ̂(t) − θ_true(t)|.

### MATLAB results so far (2026-11-09, ideal matched turret, % of time on target)
Full tables: `estimator_tests/results/sweep1_seed1.md`, `sweep2_extended_seed1.md`, `confirm_seed2.md`. 12 drone types; 30 runs each (seed 1) and 40 each (seed 2).

| Setting | Seed 1: mean / worst type / worst run | Seed 2 (fresh): mean / worst type / worst run | Blackout + recovery, worst type (seed 1 / seed 2) |
|---|---|---|---|
| CA q = 250 (best CA) | 99.5 / 97.0 / 90 | 99.6 / 98.0 / 92 | 85 / 87 |
| CA q = 4500 (old default) | 98.5 / 91.2 / 80 | — | 60 / — |
| Singer τ = 0.5, σ_m = 10 | 99.7 / 98.3 / 91 | 99.7 / 97.9 / 90 | 88 / 90 |
| Singer τ = 1, σ_m = 10 | 99.6 / 98.1 / 89 | 99.7 / 98.2 / 91 | 89 / 90 |

- Outside blackouts every tuned setting is at about 100 %. The models differ only in blackouts and the 0.5 s after.
- The worst drone type is always `blackout_double` (two blackouts in one run).
- Seed 1 favours Singer clearly; on seed 2 the two are almost tied (CA has the better worst run, Singer the better worst type).
- **Open point:** the test script reads "not much worse" as "within 2 standard errors". Because every setting sees the same drones and noise, that error is tiny (0.06 points), so even a 0.1-point gap counts as "worse". A practical tolerance (for example 0.5 points) would be fairer; the team should pick one.

## Predictor — Sanjiban, paired with Harsh

| Parameter | Proposed name in `params.m` | Meaning | Starting value or range | Status |
|---|---|---|---|---|
| Lead order | `est_pred_n` | θ_pred = θ̂ + t_p·θ̂̇, with θ̂ = atan2(y, x) and θ̂̇ = (x·v_y − y·v_x)/r², from the same state as θ̂ (D16) | **1** | **Decided** (2026-11-09, see below) |
| Second-order coefficient | `est_pred_c2` | Used only in the n = 2 fallback test (step 5 below) | t_p²/2 = 0.125 | Fallback only |

### Predictor decision: θ_pred = θ̂ + 0.5·θ̂̇ (no acceleration term)
The estimator still tracks acceleration inside its (x, y) state. That keeps θ̂ and θ̂̇ correct during turns. The acceleration just doesn't appear as an extra term in θ_pred.

**Why it wins:** a hit is decided by where the turret points **now** and where the drone is **now**. Harsh's turret path behaves like 1/(1 + 0.5 s), which is the exact opposite of our "+ 0.5·θ̂̇" lead. So turret = (θ̂ + 0.5·θ̂̇) passed through 1/(1 + 0.5 s) = θ̂, our **current** estimate, built from the newest radar reading. The 0.5 s forecast error never reaches the turret.

**Test (Python, 40 runs per drone type, 8 types incl. blackouts):**

| What | Acceleration (CA) estimator | Singer estimator |
|---|---|---|
| A pure forecast of the drone 0.5 s ahead, within 2 m: velocity only (mean / worst) | 62 % / 56 % | 71 % / 56 % |
| Same, with the acceleration term added | 43 % / 37 % | 72 % / 50 % |
| **Turret on target with the matched chain, velocity-only lead** | **99 % / 94 %** | **100 % / 98 %** |
| Same, acceleration term in the lead (and matched) | 98 % / 92 % | 95 % / 86 % |

- Even the best pure forecast is within 2 m only about 60–70 % of the time: unknown future maneuvers plus noisy speed, multiplied by 0.5 s. A turret that aims at a real forecast is limited by that.
- The acceleration term makes things worse or no better, on both measures, because estimated acceleration is noisy. Under the worst-case rule it loses clearly.
- Higher-order lead also forces Harsh to copy our coefficient exactly. With n = 1 he only needs t_p = 0.5.

**Interface for Harsh (D2):** we send θ_pred = θ̂ + 0.5·θ̂̇. The whole reference→turret path must behave like 1/(1 + 0.5 s). How he builds that is his choice. We output θ_pred **unclamped**; the −10°…85° clamp is Harsh's (D6). The Cartesian forecast atan2(p + v·0.5) can still be plotted as "where the drone will be", but it is not the reference.

## Owned by Harsh (listed here only so the interface stays consistent)

| Item | Note |
|---|---|
| Post-blackout smoothing of the reference | D5: the turret side absorbs the recovery jump. Smoothing θ_pred on our side would break the exact pairing, so we don't. |
| Reference clamp to −10°…85° | D6 |
| Fire gate | Q5. Two candidate forms: (a) docx: r̂·\|θ_turret − θ̂\| + k·σ_cross ≤ R_eng − margin; (b) our proposal: P_hit = Φ((2 − d)/σ_cross) − Φ((−2 − d)/σ_cross) ≥ threshold, with d = r̂·(θ_turret − θ̂). We supply θ̂, r̂, θ̂̇, σ_cross and time since the last reading either way. |

## How we'll tune
**Code:** `estimator_tests/test_estimator.m` runs any list of settings on the synthetic drones from `estimator_tests/make_test_drones.m` (12 drone types, all within the PDF limits) and writes tables to `estimator_tests/results/`. `estimator_tests/check_estimator.m` runs the correctness checks. Every setting sees the same drones and the same radar noise.

1. **Build a test set once:** many random maneuvers within the limits, plus the stress cases from the README (blackouts at different times, several blackouts in one run, ranges of 800 and 2000 m, sudden 20 m/s² turns).
2. **Score each candidate setting on the whole set:** per drone type, report the mean and the **worst** case of the shot hit chance, plus runs with at least one hit, time of the first hit and miss distance.
3. **Pick one setting for everything** with the worst-case rule above, then mark it Frozen and copy it into `params.m`.
4. **Confirm on a fresh random set** so the choice isn't overfitted to one set.
5. **⚠ Re-check `est_pred_n` in the full MATLAB chain.** n = 1 was chosen from a Python test with an ideal turret. If it is clearly bad in any case (especially the worst case), test n = 2 with c = 0.125 (CA) or τ²(e^(−t_p/τ) − 1 + t_p/τ) (Singer), and the turret path matched to 1/(1 + 0.5 s + c s²).

## Change log
| Date | Change |
|---|---|
| 2026-11-09 | Created: estimator, predictor, smoothing and firing-gate parameters |
| 2026-11-09 | Aligned with Jithu's doc rev 3: Singer knobs, worst-case selection rule, lead order and pairing rule, ready threshold, sweep overrides; smoothing, clamp and fire gate marked as Harsh's |
| 2026-11-09 | Predictor decided: θ_pred = θ̂ + 0.5·θ̂̇ (n = 1); `pred_c2` removed |
| 2026-11-09 | Default set to Singer τ = 1 s, σ_m = 10 m/s² (Sanjiban's choice); CA backup q = 250 |
| 2026-11-09 | `estimator_step.m` written: `est_q_ca` now continuous-time; added `est_min_updates`, `est_coast_gap`; new-reading rule; NaN before first reading. `est_pred_n`/`est_pred_c2` kept only for the n = 2 fallback test |
