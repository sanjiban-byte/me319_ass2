# Notes for running our code with an evaluation script

No instructor evaluation script or evaluation cases were provided, so we could not test against them. We did not know how the evaluator would call `turret_controller` (fixed step, ODE solver, call rate, how `meas_valid` behaves). The controller was therefore written to give the same result under any reasonable calling pattern. This note lists what it assumes and how it behaves.

## Setup

- Put the repository root on the MATLAB path. `turret_controller.m` needs `params.m`, `estimator_step.m`, `reference_shaper.m`, `prefilter_design.m`, `feedback_step.m`, `fire_logic.m` and `turret_debug.m` from the same folder. Base MATLAB only.
- Units are SI with angles in radians. θ is the elevation from the horizontal, positive up; the turret and radar are at the origin.
- `params()` caches its values. If `params.m` is edited between runs, run `clear params`.

## How the controller keeps state between calls

- `turret_controller` has no state arguments, so it keeps its state in persistent variables (estimator, prefilter, feedback filter, fire logic).
- **A new run starts** on the first call, or when `t` jumps back by more than 0.5 s. To be safe, run `clear turret_controller` (or `clear functions`) before each new run. This is required if a new run starts less than 0.5 s before the last time of the previous run.
- **A repeated or slightly earlier `t`** (ODE-solver stages, rejected steps) returns the previous outputs and changes nothing, except that a radar sample arriving in that call is still used. The estimator has the same rule: only a jump back of more than 0.5 s resets its track.

## Call rate and timing

- Any call rate works. Internally the controller updates on a fixed 50 ms grid, t = k·0.05 s, aligned with the radar. Between grid instants it holds `V_m` and `fire_cmd`.
- If the evaluator calls more slowly than every 50 ms, every call is an update using the actual time step.
- The estimator brings its state forward to the call time, so its outputs are valid at the current `t`, not at the last radar sample.

## Radar input (`meas_valid`)

- We treat `meas_valid = 1` as "this call carries a new radar sample". When `meas_valid = 0`, `r_meas` and `theta_meas` are ignored (they may be NaN).
- A sample is used on the first call that delivers it with `meas_valid = 1` and a time later than the last sample used. An exact repeat of the last values is ignored by the estimator. Both conventions work:
  - `meas_valid = 1` only at the radar instants (as in our generator);
  - the last sample held with `meas_valid = 1` until the next one.
- A sample is assumed to be taken at the time of the call that delivers it. If an evaluator delivers samples late, the estimate is offset by that delay.

## Outputs

- **`V_m`** is limited to ±600 V inside the controller. The assignment gives no voltage limit; 600 V is our assumption (typical industrial servo bus). The plant model has no limit of its own.
- **`fire_cmd`** is 1 for one 50 ms update and then 0, so every shot is a single 0→1 edge. Edges are at least 1.01 s apart (1.05 s on the 50 ms grid), so float time stamps can never make a 1.0 s spacing read as 0.9999 s.
- **Firing rule:** fire when the track is valid, the estimated miss |d̂| < 2 m and the hit probability P_hit ≥ 0.95. The turret angle passed in is taken as exact at the firing instant, and the laser is assumed instantaneous.
- **`theta_dot_turret`** is accepted but not used by the final design.

## Start of a run and turret limits

- The initial turret angle can be anything; the controller reads `theta_turret` on every call.
- `V_m = 0` until the track is valid (5 radar samples, 0.20 s after the first one). After that the turret slews at full voltage, so the first hit depends mostly on the initial angle: up to 2.55 s from the +85° stop.
- The reference is clamped to -10°…85°. Our `turret_plant.m` also has hard stops there. If the evaluator uses its own plant, the clamp keeps the reference inside the travel.

## How we scored hits (our harness, `run_scenario.m`)

- The plant is integrated with RK4 at 1 ms, and the controller is called every 50 ms.
- A pulse is a 0→1 edge of `fire_cmd` at least 1 s after the previous pulse.
- It hits if the perpendicular distance from the beam ray to the true drone at that instant is ≤ 2 m. If the drone is behind the turret, the distance to the origin is used.
- `tools/test_call_robustness.m` runs Scenario C with a solver-like call pattern (stage times, repeated times, small steps back). It gives the same pulse times as clean 50 ms calls.

## Minimal evaluation loop

```matlab
addpath(pwd); addpath('tools');
clear turret_controller
p = params();  h = 1e-3;  z = [deg2rad(37.5); 0; 0];      % [theta; theta_dot; i]
for k = 0:round(p.t_end/h)
    t = k*h;
    [rm, tm, mv] = target_motion_generator(t, 'C');
    [V, fire] = turret_controller(t, rm, tm, mv, z(1), z(2));
    f = @(zz) plant_rhs(t, V, zz);                       % tools/plant_rhs.m wraps turret_plant
    k1 = f(z); k2 = f(z + h/2*k1); k3 = f(z + h/2*k2); k4 = f(z + h*k3);
    z = z + h/6*(k1 + 2*k2 + 2*k3 + k4);
end
```
