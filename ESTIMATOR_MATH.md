# Estimator and Predictor: Math Pipeline

This document covers the estimator and predictor (`estimator_step.m`) and the firing decision built on its outputs. The equations are in the order the code runs them, and the document ends with a glossary of every variable (what goes in, what comes out, what it means).

How these are tested and scored is in `TESTING_VALIDATION_METRICS.md`.

Turret at the origin, x horizontal, y up, angles in radians from the horizontal (positive up). SI units throughout.

---

## Overview

```
 radar reading (r_m, θ_m, meas_valid) at time t
        │
   [A1] is it a new reading?
        │
   [A2] time update: move the state x̂, P forward from the last call to t
        │
   [A3] first reading?  ──yes──►  [A4] start the track from this reading
        │ no (and new)
   [A5] convert (r_m, θ_m) to (x, y) with its noise R
        │
   [A6] Kalman update: correct x̂, P with the reading
        │
   [A7] angles from the (x, y) state: θ̂, r̂, θ̂̇, θ̂̈, σ_cross
        │
   [A8] predictor: θ_pred = θ̂ + t_p·θ̂̇
        │
   [A9] flags: valid, coasting
        │
        ▼
   est struct ──► prefilter + clamp ──► feedback loop ──► turret
        │         [A10] the whole turret path must behave like 1/(1 + t_p s)
        │
   [A11] fire if valid, |d̂| < R_eng and P_hit ≥ p*   (uses θ_turret, θ̂, r̂, σ⊥)
```

Steps A2 to A9 run on **every** controller call. Steps A4 to A6 run only when a new reading arrives (20 Hz).

---

## A0. What the radar gives us (measurement model)

At each radar instant t_k = k·Δ_radar (Δ_radar = 0.05 s), unless the radar is blacked out:

$$
r_m = r + \sigma_r\,n_1, \qquad \theta_m = \theta + \sigma_\theta\,n_2, \qquad n_1, n_2 \sim \mathcal{N}(0, 1)\ \text{independent}
$$

where $r = \sqrt{x^2+y^2}$ and $\theta = \operatorname{atan2}(y, x)$ are the true range and elevation. `meas_valid` = 1 means "this call carries a new reading".

## A1. Is the reading new?

$$
\text{new} = (\text{meas\_valid} = 1)\ \wedge\ (r_m, \theta_m)\ \text{finite},\ r_m > 0\ \wedge\ (r_m, \theta_m) \neq (r_{\text{last}}, \theta_{\text{last}})
$$

A real noisy reading never repeats exactly, so an exact repeat means the caller is re-sending an old reading. It is ignored.

**Time going backwards.** If a call comes with a time slightly earlier than the previous call, the state is not moved backwards; a reading in that call is used at the current state time. An ODE solver does this on a rejected step. Only a jump back of more than 0.5 s means a new run and starts a new track. This is the same rule as `turret_controller` (decision D19).

## A2. Time update (predict the state forward to time t)

**State** (per axis: position, velocity, acceleration):

$$
\hat{\mathbf{x}} = [\,x,\ v_x,\ a_x,\ y,\ v_y,\ a_y\,]^T, \qquad \mathbf{P} = \text{its } 6\times6 \text{ covariance (uncertainty)}
$$

With $T = t - t_{\text{last call}}$ (if $T > 0$):

$$
\hat{\mathbf{x}} \leftarrow \mathbf{F}(T)\,\hat{\mathbf{x}}, \qquad \mathbf{P} \leftarrow \mathbf{F}(T)\,\mathbf{P}\,\mathbf{F}(T)^T + \mathbf{Q}(T)
$$

$\mathbf{F} = \operatorname{blkdiag}(F_3, F_3)$ and $\mathbf{Q} = \operatorname{blkdiag}(Q_3, Q_3)$: the x and y axes use the same 3×3 model and are independent.

### Singer model (the chosen one)

The acceleration fades back toward zero with time constant τ and gets random kicks:

$$
\dot a = -\frac{a}{\tau} + w(t), \qquad w = \text{white noise with intensity } q_c = \frac{2\sigma_m^2}{\tau}
$$

This $q_c$ makes the long-run spread of the acceleration exactly $\sigma_m$ (stationary variance $q_c\tau/2 = \sigma_m^2$).

As a continuous system, $\dot{\mathbf{s}} = A\mathbf{s} + G w$ with $\mathbf{s} = [p, v, a]^T$:

$$
A = \begin{bmatrix} 0 & 1 & 0 \\ 0 & 0 & 1 \\ 0 & 0 & -1/\tau \end{bmatrix}, \qquad G = \begin{bmatrix} 0 \\ 0 \\ 1 \end{bmatrix}
$$

Exact transition over time T:

$$
F_3(T) = e^{AT} = \begin{bmatrix} 1 & T & \tau^2\!\left(e^{-T/\tau} - 1 + T/\tau\right) \\ 0 & 1 & \tau\!\left(1 - e^{-T/\tau}\right) \\ 0 & 0 & e^{-T/\tau} \end{bmatrix}
$$

Exact noise, $Q_3(T) = \int_0^T e^{As}\,G q_c G^T e^{A^T s}\,ds$, computed with Van Loan's method:

$$
M = \begin{bmatrix} -A & G q_c G^T \\ 0 & A^T \end{bmatrix}, \quad E = e^{MT} = \begin{bmatrix} E_{11} & E_{12} \\ 0 & E_{22} \end{bmatrix}, \quad F_3 = E_{22}^T, \quad Q_3 = F_3\,E_{12}
$$

### CA model (compared, not used)

Only used in the comparison runs that led to choosing Singer. It is not part of the final design.

Constant acceleration with random jerk of intensity q:

$$
F_3(T) = \begin{bmatrix} 1 & T & T^2/2 \\ 0 & 1 & T \\ 0 & 0 & 1 \end{bmatrix}, \qquad
Q_3(T) = q\begin{bmatrix} T^5/20 & T^4/8 & T^3/6 \\ T^4/8 & T^3/3 & T^2/2 \\ T^3/6 & T^2/2 & T \end{bmatrix}
$$

Both are exact continuous-time models, so stepping twice by T/2 equals stepping once by T. That is why any controller call rate works.

## A3-A4. First reading: start the track

On the very first new reading (with z and R from A5):

$$
(\hat x, \hat y) = \mathbf{z}, \quad \hat v_x = \hat v_y = \hat a_x = \hat a_y = 0
$$

$$
\mathbf{P}_{\text{pos}} = \mathbf{R}, \quad \operatorname{var}(v_x) = \operatorname{var}(v_y) = \sigma_{v0}^2, \quad \operatorname{var}(a_x) = \operatorname{var}(a_y) = \sigma_{a0}^2, \quad \text{other entries } 0
$$

Before this, every output is NaN.

## A5. Convert the reading to (x, y)

$$
\mathbf{z} = r_m\begin{bmatrix} \cos\theta_m \\ \sin\theta_m \end{bmatrix}, \qquad
\mathbf{R} = \mathbf{U}\begin{bmatrix} \sigma_r^2 & 0 \\ 0 & (r_m\sigma_\theta)^2 \end{bmatrix}\mathbf{U}^T, \quad
\mathbf{U} = \begin{bmatrix} \cos\theta_m & -\sin\theta_m \\ \sin\theta_m & \cos\theta_m \end{bmatrix}
$$

In plain words: the error is σ_r = 1 m along the beam and r·σ_θ sideways (0.28 m at 800 m, 0.70 m at 2000 m). R rotates those two numbers into x-y.

## A6. Kalman update (correct the state with the reading)

$$
\mathbf{H} = \begin{bmatrix} 1&0&0&0&0&0 \\ 0&0&0&1&0&0 \end{bmatrix} \quad (\text{picks } x, y \text{ out of the state})
$$

$$
\boldsymbol{\nu} = \mathbf{z} - \mathbf{H}\hat{\mathbf{x}}, \quad
\mathbf{S} = \mathbf{H}\mathbf{P}\mathbf{H}^T + \mathbf{R}, \quad
\mathbf{K} = \mathbf{P}\mathbf{H}^T\mathbf{S}^{-1}
$$

$$
\hat{\mathbf{x}} \leftarrow \hat{\mathbf{x}} + \mathbf{K}\boldsymbol{\nu}, \qquad
\mathbf{P} \leftarrow (\mathbf{I} - \mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I} - \mathbf{K}\mathbf{H})^T + \mathbf{K}\mathbf{R}\mathbf{K}^T \quad (\text{Joseph form})
$$

The Joseph form gives the same answer as the textbook $(\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}$ but keeps P symmetric and positive under rounding.

## A7. Angles from the (x, y) state

$$
\hat r = \sqrt{\hat x^2 + \hat y^2}, \qquad \hat\theta = \operatorname{atan2}(\hat y, \hat x)
$$

$$
\hat{\dot r} = \frac{\hat x\hat v_x + \hat y\hat v_y}{\hat r}, \qquad
\hat{\dot\theta} = \frac{\hat x\hat v_y - \hat y\hat v_x}{\hat r^2}, \qquad
\hat{\ddot\theta} = \frac{\hat x\hat a_y - \hat y\hat a_x}{\hat r^2} - \frac{2\,\hat{\dot r}\,\hat{\dot\theta}}{\hat r}
$$

These are exact derivatives of $\operatorname{atan2}(y, x)$, not approximations.

**Sideways uncertainty** (the part of the position error that can make us miss):

$$
\mathbf{u}_\perp = \begin{bmatrix} -\sin\hat\theta \\ \cos\hat\theta \end{bmatrix}, \qquad
\sigma_\perp = \sqrt{\mathbf{u}_\perp^T\,\mathbf{P}_{\text{pos}}\,\mathbf{u}_\perp}
$$

## A8. Predictor

$$
\theta_{\text{pred}} = \hat\theta + t_p\,\hat{\dot\theta} \qquad (n = 1,\ t_p = 0.5\ \text{s})
$$

$$
\text{n = 2 fallback (flagged, not used):}\quad \theta_{\text{pred}} = \hat\theta + t_p\,\hat{\dot\theta} + c_2\,\hat{\ddot\theta}, \quad c_2 = t_p^2/2
$$

It is not clamped; the −10°…85° clamp is Harsh's (D6).

For plots only, the Cartesian forecast of where the drone will be: $\hat{\mathbf{x}}(t + t_p) = \mathbf{F}(t_p)\,\hat{\mathbf{x}}$.

## A9. Flags

$$
\text{valid} = \big(N_{\text{upd}} \ge N_{\min}\big) \wedge \big(\sigma_\perp \le \sigma_{\text{valid}}\big) \quad \text{(once true, stays true until reset)}
$$

$$
\text{coasting} = \big(t - t_{\text{last reading}}\big) > \Delta_{\text{coast}}
$$

## A10. Why this predictor works (the pairing with the turret)

Harsh and Jithu build the turret path so that, from θ_pred to the turret angle, it behaves like

$$
H(s) = \frac{1}{1 + t_p s}
$$

Our predictor in Laplace form is $\Theta_{\text{pred}}(s) = (1 + t_p s)\,\hat\Theta(s)$. So:

$$
\Theta_{\text{turret}}(s) = H(s)\,\Theta_{\text{pred}}(s) = \frac{1 + t_p s}{1 + t_p s}\,\hat\Theta(s) = \hat\Theta(s)
$$

**The turret ends up pointing at our current estimate θ̂(t).** The 0.5 s lead and the 0.5 s lag cancel, so the error of forecasting 0.5 s ahead never reaches the turret.

**How exact is the cancellation in the real chain?** The path is a prefilter F(z) followed by the sampled feedback loop T(z) (gain K_v = 8, notch, 50 ms updates). Writing the chain as a Taylor series in s:

$$
F\,T\,P = 1 + k_1 s + k_2 s^2 + k_3 s^3 + \dots, \qquad \theta_{\text{turret}} \approx \hat\theta + k_1\,\hat{\dot\theta} + k_2\,\hat{\ddot\theta} + \dots
$$

- **The original first-order prefilter** F = 1/(1 + τ_f s), with τ_f = t_p − 1/K_v = 0.375 s, makes k₁ = 0 but leaves k₂ ≈ −0.046 s². That is a pointing error of about 0.046·r·θ̈ (decision D33): 0.43 m RMS in Scenario B, and about 2.2 m at the corner of the flight envelope (800 m, 150 m/s crossing, 20 m/s²).
- **The prefilter now has one zero** (decision D38): y_k = α·y_{k−1} + β·u_k + β₁·u_{k−1}. Its three coefficients are chosen so that k₀, k₁ and k₂ all match, which makes k₂ = 0. For an ideal loop this is
$$
F(s) = \frac{1 + s/K_v}{1 + t_p s}.
$$
  On the real sampled loop at 50 ms the design gives a pole time constant of 0.521 s and a zero time constant of 0.145 s. What is left is k₃ ≈ −0.0044 s³, ten times smaller. `test_shaper.m` checks this: with a constant-acceleration θ̂, the steady turret error falls from −0.046·θ̈ to −5×10⁻¹⁰ rad.

So with the zero, the turret points at θ̂(t) up to a third-order term. The predictor stays first order (n = 1); only the turret side changed.

A hit happens when the sideways miss is within the engagement radius:

$$
d = r\,\sin(\theta_{\text{turret}} - \theta) \approx r\,(\theta_{\text{turret}} - \theta), \qquad \text{hit if } |d| \le R_{\text{eng}} = 2\ \text{m}
$$

## A11. Firing decision (built on our outputs)

**Rule:** request a laser pulse when the track is ready, our estimated miss is inside the radius, **and** the chance of a hit is high enough:

$$
\text{fire} = \text{valid}\ \wedge\ \big(|\hat d| < R_{\text{eng}}\big)\ \wedge\ \big(P_{\text{hit}} \ge p^*\big)
$$

p* is the one threshold. It is fitted on the training set and checked on the test set (see `TESTING_VALIDATION_METRICS.md`); the fitted value is **p* = 0.95**. How a request becomes a pulse is the PDF's rule: a pulse fires on a 0→1 change of `fire_cmd`, and only if at least t_recharge = 1 s has passed since the last pulse (the code keeps 1.01 s, so floating-point time stamps can never make it 0.9999 s).

This is the whole rule in `fire_logic.m` (decision D39). The earlier extra conditions are dropped: the turret-rate match |θ̇_turret − θ̂̇| ≤ 0.02 rad/s, and the elevation-range check, which is implied anyway because the turret cannot point outside its stops.

### Step 1. Our estimated miss

The turret angle θ_turret is known exactly (it is an input to `turret_controller`), and the laser is instant. So our best guess of the sideways miss right now is:

$$
\hat d = \hat r\,\sin(\theta_{\text{turret}} - \hat\theta) \;\approx\; \hat r\,(\theta_{\text{turret}} - \hat\theta)
$$

This is the angle gap times the range, which turns an angle into metres. Only the sideways miss matters, because the beam is a straight line: an error along the beam cannot cause a miss.

### Step 2. The true miss is our estimate plus our error

The true miss uses the true angle θ:

$$
d = r\,(\theta_{\text{turret}} - \theta) = \underbrace{r\,(\theta_{\text{turret}} - \hat\theta)}_{\approx\ \hat d} + \underbrace{r\,(\hat\theta - \theta)}_{=\ \varepsilon}
\qquad\Rightarrow\qquad d \approx \hat d + \varepsilon
$$

Replacing r by r̂ in the first term changes it by at most |r − r̂|·|θ_turret − θ̂| ≈ 1 m × 2/800 rad ≈ 2.5 mm, which is negligible. **ε is our estimate's sideways error, in metres.**

### Step 3. How big ε is: σ⊥ from the Kalman filter

To first order, a sideways position error of the estimate is ε ≈ u⊥ᵀ(p̂ − p), where p̂ − p is the (x, y) position error and u⊥ is the unit vector across the beam (A7).

The Kalman filter's P is the covariance of the state error. So the variance of ε is

$$
\operatorname{var}(\varepsilon) = \mathbf{u}_\perp^T\,\mathbf{P}_{\text{pos}}\,\mathbf{u}_\perp = \sigma_\perp^2
$$

which is exactly `est.sigma_cross`. The radar noise is Gaussian and the filter is linear, so ε is Gaussian with mean 0:

$$
\varepsilon \sim \mathcal{N}(0,\ \sigma_\perp^2)
$$

**Checked in simulation** across 12 drone types: the real errors fell within ±σ⊥ 69-72% of the time and within ±2σ⊥ 96-97% (Gaussian: 68% and 95%). During blackouts the figures were 65-77% and 94-98%, while σ⊥ grew to about 2.2 m. So σ⊥ is honest.

### Step 4. Write "hit" in terms of ε

A hit means the true miss is within the radius:

$$
|d| \le R_{\text{eng}} \iff -R_{\text{eng}} \le \hat d + \varepsilon \le R_{\text{eng}} \iff -R_{\text{eng}} - \hat d \;\le\; \varepsilon \;\le\; R_{\text{eng}} - \hat d
$$

### Step 5. Probability that a Gaussian lands in that range

For a Gaussian with mean 0 and spread σ, the chance of landing between a and b is Φ(b/σ) − Φ(a/σ). Dividing by σ turns metres into "how many σ away". So:

$$
\boxed{P_{\text{hit}} = \Phi\!\left(\frac{R_{\text{eng}} - \hat d}{\sigma_\perp}\right) - \Phi\!\left(\frac{-R_{\text{eng}} - \hat d}{\sigma_\perp}\right)}
$$

$$
\Phi(z) = \int_{-\infty}^{z} \frac{1}{\sqrt{2\pi}}\,e^{-s^2/2}\,ds = \tfrac12\left[1 + \operatorname{erf}\!\left(z/\sqrt2\right)\right]
$$

Φ(z) is the chance that a standard Gaussian value comes out below z. Some reference values: Φ(0) = 0.5, Φ(1) = 0.841, Φ(2) = 0.977. The `erf` form is available in base MATLAB.

- The **first term** is the chance the drone is not past the far edge of the circle.
- The **second term** is the chance it is past the near edge, which is subtracted.

**Examples** (R_eng = 2 m):

| d̂ | σ⊥ | P_hit | Distance check alone |
|---|---|---|---|
| 1.0 m | 0.4 m | 0.994 | fire |
| 1.5 m | 0.4 m | 0.894 | fire |
| 1.9 m | 0.4 m | 0.599 | fire (a near coin flip) |
| 1.5 m | 0.7 m (far range) | 0.762 | fire |
| 0 m | 2.2 m (blackout) | 0.637 | fire (blind shot) |

### Step 6. How the two checks relate

If |d̂| ≥ R_eng, one edge of the circle is on or behind the estimate, so P_hit ≤ Φ(0) = 0.5. So **for any p* ≥ 0.5, P_hit ≥ p* already implies |d̂| < R_eng.** The distance check is still kept on purpose: it is the plain, readable condition from the PDF, and it guards the rule if p* were ever set below 0.5.

**What the P_hit term adds:** it holds fire when we are close to the edge of the circle, or when we are unsure (σ⊥ large, as in a blackout). In both cases a miss would waste the 1 s recharge.

### Assumptions behind P_hit

| Assumption | Why it holds |
|---|---|
| θ_turret is exact | It is an input to `turret_controller` |
| The laser is instant | Stated in the PDF |
| r̂ ≈ r | Range error ≈ 1 m out of 800-2000 m (Step 2) |
| sin x ≈ x | Angle gaps that matter are below 0.003 rad |
| ε is Gaussian, mean 0, spread σ⊥ | Gaussian radar noise and a linear filter; spread checked in simulation (Step 3); mean checked by the bias test in `TESTING_VALIDATION_METRICS.md` |

---

## Variable glossary

**In** = input to `estimator_step`, **Out** = in the `est` struct, **Int** = internal.

| Symbol | Code name | Meaning | Units | Role |
|---|---|---|---|---|
| t | `t` | Current time of this controller call | s | In (from the evaluation script via `turret_controller`) |
| r_m | `r_meas` | Measured range | m | In (radar) |
| θ_m | `theta_meas` | Measured elevation | rad | In (radar) |
| meas_valid | `meas_valid` | 1 = this call carries a new radar reading | - | In (radar) |
| r, θ | - | True range and elevation (unknown to us) | m, rad | Only in tests |
| n₁, n₂ | - | Standard normal noise draws | - | Only in tests |
| x̂ | `S.x`, out as `est.state` | State estimate [x, v_x, a_x, y, v_y, a_y] | m, m/s, m/s² | Int (also Out for plots) |
| P | `S.P` | Covariance of the state estimate (how unsure we are) | mixed | Int |
| T | `dt` | Time since the last call | s | Int |
| F, F₃ | `F`, `F3` | State transition over time T | - | Int |
| Q, Q₃ | `Q`, `Q3` | Uncertainty added by unknown maneuvers over time T | mixed | Int |
| A, G, M, E | - | Continuous Singer model and Van Loan matrices | - | Int |
| q_c | `qc` | Singer noise intensity, 2σ_m²/τ | m²/s⁵ | Int |
| z | `z` | Reading converted to (x, y) | m | Int |
| R | `R` | Covariance of z | m² | Int |
| U | `Rot` | Rotation from (along beam, sideways) to (x, y) | - | Int |
| H | `H` | Picks x and y out of the state | - | Int |
| ν | `innov` | Innovation: reading minus predicted position | m | Int |
| S | `Sm` | Expected spread of ν | m² | Int |
| K | `K` | Kalman gain: how much to trust the reading | - | Int |
| r̂ | `est.r_hat` | Estimated range now | m | Out → fire rule |
| θ̂ | `est.theta_hat` | Estimated elevation now | rad | Out → fire rule, prefilter start |
| ṙ̂ | `rd` | Range rate | m/s | Int |
| θ̂̇ | `est.theta_dot_hat` | Elevation rate | rad/s | Out → Harsh (prefilter start) |
| θ̂̈ | `est.theta_ddot_hat` | Elevation acceleration | rad/s² | Out (n = 2 fallback, plots) |
| u⊥ | `u` | Unit vector across the beam | - | Int |
| σ⊥ | `est.sigma_cross` | 1-σ sideways position uncertainty now | m | Out → fire rule |
| θ_pred | `est.theta_pred` | Predicted elevation reference, unclamped | rad | Out → Harsh (prefilter input) |
| n | `est.n` | Predictor order used (1) | - | Out → Harsh (prefilter must match) |
| x̂(t + t_p) | `est.pos_pred` | Cartesian forecast 0.5 s ahead | m | Out (plots only) |
| N_upd | `est.n_updates` | Readings used so far | - | Out |
| t − t_last | `est.t_since_meas` | Time since the last reading used | s | Out |
| valid | `est.valid` | Track has settled; safe to start the prefilter and fire | - | Out → Harsh |
| coasting | `est.coasting` | No reading for longer than Δ_coast (blackout) | - | Out → logging |
| H(s) | - | Required response from θ_pred to the turret angle | - | Harsh and Jithu's side |
| d | - | True sideways miss of the beam (unknown to us) | m | Hit condition |
| θ_turret | `theta_turret` | Turret elevation, known exactly | rad | In to `turret_controller`; used by the fire rule |
| d̂ | - | Our estimated sideways miss, r̂·sin(θ_turret − θ̂) | m | Fire rule |
| ε | - | Sideways error of our estimate, d − d̂ | m | Fire rule (statistics) |
| Φ | - | Standard normal cumulative distribution | - | Fire rule |
| P_hit | - | Probability the pulse hits, given d̂ and σ⊥ | - | Fire rule |
| p* | - | Firing threshold on P_hit (fitted on train, checked on test) | - | Fire rule |
| R_eng | `p.R_eng` | Engagement radius, 2 m (PDF) | m | Hit condition, fire rule |
| fire | `fire_cmd` | 1 = request a pulse (fires on a 0→1 change after the 1 s recharge) | - | Out of `turret_controller` |

---

## References

- R. A. Singer, "Estimating optimal tracking filter performance for manned maneuvering targets," *IEEE Trans. Aerospace and Electronic Systems*, AES-6(4), 1970. (Singer model, σ_m and τ guidance)
- C. F. Van Loan, "Computing integrals involving the matrix exponential," *IEEE Trans. Automatic Control*, 23(3), 1978. (Exact Q)
- Y. Bar-Shalom, X. R. Li, T. Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley, 2001. (Kalman filter, converted measurements, CA/white-noise-jerk model, Joseph form)
- X. R. Li, V. P. Jilkov, "Survey of maneuvering target tracking. Part I: Dynamic models," *IEEE Trans. Aerospace and Electronic Systems*, 39(4), 2003. (Comparison of CA and Singer models)
