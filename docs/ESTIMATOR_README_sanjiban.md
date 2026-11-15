# ME319 Assignment 2: Predictive Tracking and Turret Control

Mumbai 2044: a laser turret has to hit a maneuvering V-44 drone using noisy radar readings. The full problem statement is in [Assignment_2_ME_319_Control_Systems_S3.pdf](Assignment_2_ME_319_Control_Systems_S3.pdf).

**Deadline:** November 10, 2026, 11:59 PM

> ### Note for Claude (or any AI assistant used by a team member)
> - **Do not make crucial design decisions on your own.** Model choices, tuning values, interfaces between team members, and anything another member depends on are the user's (and the team's) call.
> - **Always ask before choosing a design.** Lay out the options, with their trade-offs and test evidence, in simple words, and let the user decide.
> - **Explain everything nicely.** Assume the user wants to understand why, not just get code.
> - **Don't silently change decisions already recorded** in this README, `TUNABLE_PARAMETERS.md`, or the team's work-split document. If something looks wrong, flag it to the user instead of fixing it yourself.

## The 4 Parts of the Assignment

| # | Part | Takes in | Gives out |
|---|---|---|---|
| 1 | **Estimation and Prediction** | Radar readings `r_meas`, `theta_meas`, `meas_valid` | Where the drone is now, where it will be 0.5 s later, the angle the turret should aim at, and how confident we are |
| 2 | **Turret Control** | The aim angle from Part 1 and the turret's current angle and speed | Motor voltage `V_m`, plus the turret model `Gp(s)` and the stability analysis |
| 3 | **Firing and Integration** | The outputs of Parts 1 and 2 | `fire_cmd` (when to fire), and the final `turret_controller.m` that combines everything |
| 4 | **Simulation and Testing** | The function definitions given in the PDF | The target generator, the turret simulation, the hit checker, the scores, and the stress tests |

Parts 1, 2 and 4 can be worked on in parallel. Part 2 can be tested with a fake aim angle (a straight ramp plus some noise), and Part 1 can be tested without any turret at all.

## Important Points and Clarifications

### 1. Handling noisy readings

For the noisy radar readings, we have considered a **Kalman filter** and the simpler **fixed-gain alpha-beta-gamma filter**.

### 2. Vanishing and dodging

- **Vanishing:** the drone is still physically there and **can still be hit**. Only the radar goes blind. During the blackout, `meas_valid = 0` and we keep predicting without new readings.
- **Dodging:** the PDF only says the maneuver is **smooth and stays within the limits** (speed ≤ 150 m/s, acceleration ≤ 20 m/s², range 800 to 2000 m). It does not say how the drone decides to dodge.

### 3. The turret's lag does not have to be exactly 0.5 s

The laser travels instantly, so at the moment we fire, the turret must point at where the drone is **right now**.

Our aim angle is where the drone will be **0.5 s from now**. If the turret takes L seconds to catch up with the aim angle, then right now it points at where the drone will be **0.5 − L seconds from now**. For that to be "right now", the **total delay from the aim angle to the turret must add up to 0.5 s**.

The turret by itself does not have to be that slow. Some ways to get a total of 0.5 s:

- **Way 1: one slow turret.** Make the turret itself slow enough to lag 0.5 s. This is simple, but it follows curves badly and lets more noise through.
- **Way 2: fast turret plus smoothing.** Make the turret fast (for example 0.07 s), then add smoothing steps before it to make up the remaining 0.43 s. The smoothing also cleans up the noise. This is the team's chosen design (decision D4 in the work-split document).
- **Way 3: fast turret with nothing added.** This is wrong. The turret points ahead of the drone and misses by tens of metres.

In a quick test, Way 2 kept the turret on target about 85–98% of the time, Way 1 about 58–67%, and Way 3 almost never. (These are named "ways" so they don't get confused with the work-split document's Options A and B, which mean something else.)

**Matching rule (decision D2):** the smoothing must exactly undo the prediction's lead. Our prediction is θ_pred = θ̂ + 0.5·θ̂̇ (current angle plus 0.5 s × angular speed), so the whole path from reference to turret must behave like 1/(1 + 0.5 s). Then the lead and the lag cancel, and the turret ends up pointing at our **current** best estimate of the drone, built from the newest radar reading.

This beats aiming at a true 0.5 s forecast: in a quick test, even the best forecast was within 2 m only about 60–70% of the time, while the matched turret was on target about 94–100% of the time. Adding an acceleration term to the lead did not help. The estimator still tracks acceleration internally, so θ̂ and θ̂̇ stay correct during turns.

**Decision: the predictor uses n = 1 only** (θ̂ + 0.5·θ̂̇, no acceleration term).

> **⚠ Flag for testing:** this was decided from a Python test with an ideal turret (no voltage limit, turret starting aligned). Once the full MATLAB chain runs, re-check it. If n = 1 turns out clearly bad in any test case (especially the worst case), test **n = 2** (θ̂ + 0.5·θ̂̇ + c·θ̂̈, with c = 0.125 for the acceleration model, or the matching c for Singer). The turret path would then have to match 1/(1 + 0.5 s + c s²).

### 4. The −10° to +85° limit

The PDF only gives this as a mechanical limit and does not explain it. One likely reason is that the turret sits on a raised place, like a rooftop, so it can look slightly downward.

**So, with the turret as the origin, do not assume y > 0 for the drone.** Use `atan2(y, x)` for angles, and do not fire if the needed angle is outside −10° to +85°.

### 5. What is random and what is not

| Effect | What we know | How to handle it |
|---|---|---|
| **Radar noise** | Random, zero mean, known size: σ_r = 1 m, σ_θ = 0.02° | The filter's measurement noise |
| **Drone maneuver** | Unknown. It does not come from any given probability distribution, so it is basically impossible to model exactly. Only its limits are known. | Treat it as "unknown push" (process noise) in the filter and tune it |
| **Radar loss** | Unknown timing (lasts ≤ 0.5 s and starts at or after 1 s), not from any given distribution. But `meas_valid` tells us when it happens. | Skip updates and keep predicting |

### 6. Why we cannot use raw radar readings directly

The hit radius is 2 m, so the allowed angle error gets smaller as the drone gets farther away. The angle noise of 0.02° also turns into more metres of error farther away.

| Distance | Allowed angle error | Sideways error from noise | Error if we estimate speed from 2 raw readings and predict 0.5 s ahead |
|---|---|---|---|
| 800 m | ±0.143° | 0.28 m | about 3.9 m |
| 2000 m | ±0.057° | 0.70 m | about 9.9 m |

- **Where the drone is now:** raw readings are good enough. At 2000 m the noise is 0.7 m, well inside the 2 m radius.
- **Where the drone will be 0.5 s later:** we need its speed. Speed from 2 raw readings, 0.05 s apart, is very noisy because a small position error divided by 0.05 s becomes a big speed error. In a quick test, predicting this way stayed on target only about 21% of the time.
- **Why we must predict:** in 0.5 s the drone can move up to 75 m (at 150 m/s), so pointing at where it is now does not work.
- **Range noise barely matters:** the laser is a straight line, so an error along the beam doesn't cause a miss. Only the sideways error matters.

### 7. Sanjiban's idea: how many derivatives to use (Taylor series intuition)

Predicting 0.5 s ahead is like a Taylor series: position + velocity × t + ½ × acceleration × t² + …

- **Too few terms:** we ignore real motion. Ignoring an acceleration of 20 m/s² alone costs ½ × 20 × 0.5² = **2.5 m**, which is already more than the 2 m radius.
- **Too many terms:** each higher derivative is much noisier to estimate. For example, acceleration from raw readings has about 680 m/s² of noise at 2000 m.
- **So there is a best stopping point.** In a quick test, using **position, velocity and acceleration** worked best. Adding jerk made things worse, and stopping at velocity was worse for slow weaves.

Also, **(x, y) is easier to work with than (r, θ)** for the drone. A drone flying in a straight line looks curved in (r, θ), and the speed and acceleration limits are given in x and y. So we convert each radar reading to (x, y) first.

### 8. Stress testing and scenarios

The PDF gives **no** test data. It only describes Scenarios A (no maneuver), B (maneuver from t = 1 s) and C (0.5 s radar loss from t = 1 s). We have to write the target generator ourselves.

The instructor's evaluation cases arrive only before the deadline, and **we are not allowed to retune for them**. So we need to think of and generate our own tough cases, for example:

- The drone near the limits: 800 m, 2000 m, and 150 m/s
- Sudden turns at the full 20 m/s², including right when the radar goes blind
- Radar loss at different times, and more than one loss in a single run
- Fast crossing paths close to the turret
- The drone near +85° or below the turret's level (y < 0)
- The turret starting far from the drone's angle

## Files

- `Assignment_2_ME_319_Control_Systems_S3.pdf`: the assignment
- `TUNABLE_PARAMETERS.md`: every design parameter we tune, its value, how it was chosen, and the test results behind it
- `estimator_step.m`: estimator and predictor (Part 1). Called once per controller call; returns the current target estimate, the predicted elevation reference `theta_pred` and its uncertainty.
- `estimator_tests/`: tests for Part 1
  - `make_test_drones.m`: synthetic drone paths (12 types, all within the PDF limits, including blackouts)
  - `test_estimator.m`: scores any list of estimator settings on those drones and writes result tables
  - `check_estimator.m`: quick correctness checks (prints PASS/FAIL)
  - `run_estimator_sweeps.m`: reproduces every table in `estimator_tests/results/`
  - `results/`: the result tables (`.md`)
- Still to be added by the team: `params.m`, `turret_plant.m`, `turret_controller.m`, `target_motion_generator.m` and the full-system scripts

## How to reproduce

Requires MATLAB (tested on R2025a, no toolboxes). `params.m` must be in the repository root, because every file reads its constants from it.

Estimator and predictor (Part 1), from the repository root in MATLAB:

```matlab
cd estimator_tests
check_estimator          % correctness checks, about 10 s
run_estimator_sweeps     % all result tables in estimator_tests/results/, about 10 min
```
