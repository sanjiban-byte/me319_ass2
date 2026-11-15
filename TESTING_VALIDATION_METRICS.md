# Testing, Validation Sets and Metrics

How the design is tuned, checked and finally judged, and which numbers we report.

The math of the estimator, predictor and firing rule is in `ESTIMATOR_MATH.md`. This document only covers the data sets and the metrics.

**Ground rules (from the PDF):**
- The design is tuned **once** and then frozen.
- The instructor's evaluation cases are never used for tuning, and no scenario gets its own retuning.
- The PDF's standard Scenarios A, B and C are always reported separately, as the PDF requires.
- All our data is synthetic, made by the team's target generator, and every path is checked numerically against the PDF limits:
  - range 800-2000 m
  - speed ≤ 150 m/s
  - acceleration ≤ 20 m/s²
  - elevation −10° to +85°
  - each blackout ≤ 0.5 s, starting at t ≥ 1 s

---

## 1. The three data sets

One **run** is one 10 s engagement: one drone path plus its own radar noise. Volumes are counted in runs.

| Set | Used for | Volume | Drone scenarios | May the design change after seeing it? |
|---|---|---|---|---|
| **Train** | Fitting every tuned value: estimator settings, the firing threshold p*, and any other swept parameter | N_train (whatever the team's training sweeps used) | The training scenario types | Yes. This is where all tuning happens. |
| **Test** | Checking that the tuned values (thresholds, parameters) still hold on new drones | **N_test ≈ 2/5 × N_train** | The same kinds as train, **plus 2-3 extra types** that are similar or a bit different | No retuning on it. If a value clearly fails here, retune **on train** with the reason written down, then re-check. |
| **Validation** | Final judgment of the frozen design, on new kinds of scenarios | Fixed before generating (see Section 2) | The four validation scenarios V1-V4 (Section 2) | No. Results are reported as they come out. |

### Volume rule for the test set

$$
N_{\text{test}} = \operatorname{round}\!\left(\tfrac{2}{5}\,N_{\text{train}}\right)
$$

- Spread N_test across the train types in about the same proportions as in train, and give the 2-3 extra types a share of their own.
- **Example only:** the first estimator sweep used 12 types × 30 runs = 360 training runs per setting. That gives N_test ≈ 144, for example about 10 runs per type over 12 + 2-3 types.

### The 2-3 extra test types

These are proposals; the team picks the final ones. They should be **similar to train but a bit different**, and should not overlap the validation scenarios.
- **Different maneuver shape:** constant-acceleration segments joined by smooth 0.2 s transitions (random turns that hold, then switch), instead of the sine-shaped weaves used in training.
- **Near the elevation limits:** the drone flying below the turret's level (elevation down to −10°) or close to +85°.
- **Speed changes along the path:** acceleration along the flight direction (speeding up and slowing down) rather than turning.

### Rules for all three sets
1. **No run appears in two sets.** Each set uses its own random seeds, and the seeds are recorded with the results.
2. **Same generator code and radar noise model for all sets.** Only the scenario settings and the seeds differ.
3. **Sets are generated before anyone looks at results on them.** Test and validation are never regenerated to get nicer numbers.
4. **This guideline does not generate data.** The team generates the sets with its own generator and records the volumes used.

---

## 2. Validation scenarios

These are the four new kinds of scenarios used for final validation. Every path stays inside the PDF limits.

| ID | Scenario | What to generate | Why it is hard |
|---|---|---|---|
| **V1** | **Drone near the limits of distance and velocity** | Paths that spend long stretches within about 50 m of 800 m or of 2000 m while flying at 140-150 m/s. Include both corners: fast and close, and fast and far. Maneuvers are allowed up to a_max (turning keeps the speed under the limit). | The fastest angle rates (close) and the largest sideways noise (far) occur at top speed. |
| **V2** | **Sudden turns at full a_max, including right when the radar goes blind** | Turns at 20 m/s² with a sharp but smooth onset (0.2 s ramp), lasting 0.5-2 s, in both directions, including quick reversals (S-turns). Sub-cases: (a) the turn starts with the radar on; (b) **the turn starts exactly when a blackout starts**; (c) the turn starts in the middle of a blackout; (d) the turn starts just as the radar returns. | The estimator learns about the turn late, or only after the blackout. |
| **V3** | **Radar loss at different times, and more than one loss per run** | Single 0.5 s losses with start times stepped across the run (1.0, 1.5, …, 9.5 s). Runs with 2 and with 3 losses, including losses separated by only a short stretch of radar (for example 0.2-0.5 s). | Coasting error and recovery at every phase of a maneuver; repeated losses before the track has recovered. |
| **V4** | **Drone very close to and very far from the turret** | Close: 800-850 m, especially crossing paths (angle rate up to 150/800 ≈ 0.19 rad/s). Far: 1950-2000 m, mostly with maneuvers. | Close: the turret must follow the fastest angle changes. Far: the 2 m circle is only ±0.057° wide, and the sideways radar noise is largest (r·σ_θ ≈ 0.70 m). |

**Notes:**
- The PDF promises at most one blackout, of at most 0.5 s. The multi-loss runs in V3 go **beyond the PDF** as a stress test, so report them separately from the single-loss runs.
- **Volume:** fix it before generating, and enough per sub-case for a worst case to mean something. A suggestion: at least about 30 runs per sub-case.

---

## 3. Engagement metrics (the whole system)

These follow the PDF's grading terms: *interception success, interception time, engagement accuracy and consistency across cases.*

**Notation for run i:**
- Pulses k = 1…K_i fire at times t_k. A pulse is a 0→1 change of `fire_cmd` that the PDF's rule accepts (at least 1 s after the last pulse).
- The miss of pulse k is the true sideways distance between the beam and the drone at that instant:

$$
m_k = r(t_k)\,\big|\sin\!\big(\theta_{\text{turret}}(t_k) - \theta(t_k)\big)\big|
$$

- The pulse is a **hit** if m_k ≤ R_eng = 2 m.

### M1. Interception success (PDF definition)

$$
S_i = \begin{cases} 1 & \text{if at least one pulse in run } i \text{ hits} \\ 0 & \text{otherwise} \end{cases}
\qquad \text{success rate} = \frac{1}{N}\sum_i S_i
$$

### M2. Interception time (PDF definition)

$$
T_i = \text{time of the first hit in run } i
$$

Report the mean and the median over runs. For runs with no hit, T_i does not exist: report how many there were, and in the averages use T_i = t_end = 10 s as a penalty (stated next to the number).

### M3. Engagement accuracy

**Per-shot hit rate**, over all pulses in a scenario (per run as well: h_i = hits_i / K_i):

$$
h = \frac{\text{number of hits}}{\text{number of pulses}}
$$

**Median miss distance**, over all pulses in a scenario:

$$
\tilde m = \operatorname{median}_k\ m_k
$$

The median is used because one wild miss would drag a mean around.

### M4. Consistency (average minus worst case)

For each metric M above, the gap between the average and the worst case:

$$
\Delta_M = \big|\,\overline{M} - M_{\text{worst}}\,\big|
$$

- The worst case is the lowest value for success rate and hit rate, and the highest value for interception time and miss distance.
- **Δ is always ≥ 0, and smaller means more consistent.**
- Report Δ two ways:
  - **within a scenario**: average over runs vs. the worst run;
  - **across scenarios**: average of the scenario averages vs. the worst scenario.
- For success rate, a single run is 0 or 1, so use only the across-scenarios form.

---

## 4. Estimator metrics

**Signed sideways error** of the current estimate at each controller call, positive when the estimate is above the drone:

$$
e(t) = r(t)\,\sin\!\big(\hat\theta(t) - \theta(t)\big)
$$

It is scored from the moment `est.valid` turns true until t_end. Report it per scenario, and separately for blackout windows (the blackout plus 0.5 s after).

### E1. Bias: mean sideways error, with a t-test across runs

**Why one number per run:** inside one run, consecutive errors are strongly linked, because the filter remembers. Treating the ~2000 samples of a run as independent would make any test far too confident. Different runs use independent drones and independent radar noise, so the **runs are independent**. The test is therefore done on one mean per run.

**Per-run mean error:**

$$
\bar e_i = \frac{1}{n_i}\sum_{t} e_i(t)
$$

**Bias and its t-test** over N runs (null hypothesis: no bias):

$$
\hat b = \frac{1}{N}\sum_{i=1}^{N} \bar e_i, \qquad
s = \sqrt{\frac{1}{N-1}\sum_{i=1}^{N}\big(\bar e_i - \hat b\big)^2}, \qquad
t = \frac{\hat b}{s/\sqrt N}\quad (N-1 \text{ degrees of freedom})
$$

**Report:**
- b̂ in metres, with its 95% interval b̂ ± t_{0.975, N−1}·s/√N
- t and the two-sided p-value

**Reading it:**
- |t| below about 2 (p > 0.05) means no bias was detected.
- Also compare b̂ with R_eng = 2 m to judge whether a bias would even matter.

**Why E1 alone is not enough:** with symmetric maneuvers (turns both ways), a filter that lags in every turn can still show zero average bias. E2 catches that.

### E2. Error vs. the drone's sideways acceleration (regression)

**Regressor:** the drone's true acceleration across the line of sight:

$$
a_\perp(t) = \mathbf{u}_\perp(t)^T\mathbf{a}(t) = -\sin\theta(t)\,a_x(t) + \cos\theta(t)\,a_y(t)
$$

**Why this regressor is the most defensible choice:**
1. Only the sideways error causes misses, and a⊥ is measured in the same sideways direction as the error e.
2. Acceleration along the beam changes the range, which barely moves the angle. Acceleration across the beam is what can create sideways error.
3. Tracking-filter theory: a filter that reacts late to a maneuver has an error that grows with the acceleration it hasn't caught up with. The slope therefore measures **how much our estimate lags per unit of sideways acceleration**. A perfect estimator has slope 0.

**Model**, fitted by ordinary least squares (OLS) on all scored samples j of a scenario:

$$
e_j = \beta_0 + \beta_1\,a_{\perp,j} + \text{residual}_j
$$

$$
\hat{\boldsymbol\beta} = (\mathbf X^T\mathbf X)^{-1}\mathbf X^T\mathbf e, \qquad \mathbf X = [\,\mathbf 1\ \ \mathbf a_\perp\,]
$$

**OLS quantities to report:**

$$
\hat\sigma = \sqrt{\frac{\text{RSS}}{n-2}} \ \ (\text{residual standard error, m}), \qquad
\operatorname{SE}(\hat\beta_1) = \frac{\hat\sigma}{\sqrt{\sum_j (a_{\perp,j} - \bar a_\perp)^2}}, \qquad
t_{\text{OLS}} = \frac{\hat\beta_1}{\operatorname{SE}(\hat\beta_1)}, \qquad R^2
$$

**Units and meaning:** β₁ is in metres per (m/s²), i.e. s². For example, β₁ = 0.01 s² means a 20 m/s² turn shifts the estimate 0.2 m sideways.

**Use the run-level t-test for conclusions.** The pooled OLS standard error has the same problem as in E1: it treats linked samples as independent, so SE(β̂₁) comes out too small and t_OLS too large. For the real test:
1. Fit β̂₁,ᵢ separately in each run.
2. Leave out runs with almost no sideways acceleration (straight flight), since their slope is undefined.
3. Test the run slopes across runs, exactly as in E1:

$$
t_{\beta} = \frac{\overline{\hat\beta_1}}{s_\beta/\sqrt{N}}
$$

Report β̂₁, SE(β̂₁), t_OLS, σ̂ and R² from the pooled fit (they describe the fit), and t_β with its p-value (that one decides).

### E3. Is the uncertainty honest? (calibration)

The firing rule's P_hit trusts σ⊥ (`ESTIMATOR_MATH.md`, A11), so check it:

$$
c_1 = \%\ \text{of samples with}\ |e| \le \sigma_\perp \ (\text{target } 68.3\%), \qquad
c_2 = \%\ \text{with}\ |e| \le 2\sigma_\perp \ (\text{target } 95.4\%)
$$

Report it for normal tracking and for blackout windows separately. If c₁ and c₂ fall clearly below target, σ⊥ is too optimistic and P_hit overstates the hit chance.

### E4. Plain accuracy

- **RMS sideways error:** sqrt(mean of e²), in metres.
- **On target:** % of time with |e| ≤ R_eng.
- **Tail error:** the 99th percentile of |e|.

---

## 5. Fitting and checking the firing threshold p*

- **Fitted on train only.** The objective follows the worst-case rule: the worst scenario type matters most. Used for the 15 Nov re-run (decision D39), maximising in order:
  1. the worst scenario's success rate (M1),
  2. then the worst scenario's per-shot hit rate (M3),
  3. then the mean interception time (M2, lower is better);
  4. then the total number of hits.

  Result on Train-G: **p* = 0.95** (0.95 to 0.995 all gave zero misses; 0.95 had the earliest first hits). See `results/compare_designs.md`.
- **Checked on test.** Report M1-M4 at the fitted p*, and at p* ± 0.05 to show how sensitive the results are.
- **p* must be fitted with the real turret path in the loop** (Harsh's and Jithu's parts). With an ideal turret, d̂ stays near 0 and p* would barely matter.

---

## 6. Report layout (per data set, one row per scenario)

| Scenario | Runs | Success rate | Interception time: mean / median (no-hit runs) | Hit rate per shot | Median miss (m) | Δ success / Δ time / Δ hit rate / Δ miss | Bias b̂ (t, p) | β₁ (SE, t_OLS; t_β, p) | Calibration c₁ / c₂ |
|---|---|---|---|---|---|---|---|---|---|

The last row of each table is "all scenarios", with the across-scenario Δ's.

---

## 7. Estimator-only test (used for the model choice)

The estimator model (Singer vs CA) was chosen **before** the full turret chain existed, so it used an ideal turret that isolates the estimator.

**Ideal matched turret.** The required response 1/(1 + t_p s), stepped exactly for an input that is linear between calls (call interval Δ):

$$
a = e^{-\Delta/t_p}, \quad g = \frac{t_p}{\Delta}(1 - a), \quad
y_{k+1} = a\,y_k + u_{k+1} - a\,u_k - g\,(u_{k+1} - u_k), \quad u = \theta_{\text{pred}}
$$

It starts at the first valid estimate with y = θ̂, so there is no start-up jump.

**Score:** % of time on target, i.e. the chance that a shot fired at a random moment would hit:

$$
\text{on target} = 100\cdot\frac{1}{N}\sum_{t\,\in\,[t_{\text{valid}},\,t_{\text{end}}]}\mathbf 1\big\{\,r\,|\sin(y - \theta)| \le R_{\text{eng}}\big\}
$$

**Per setting:**
- **mean** is the average over drone types.
- **worst type** is the lowest type average.
- **worst run** is the lowest single run.

**CA vs Singer** was compared with paired per-run differences d_i on the same drones and noise: mean d̄ and SE = std(d_i)/√N.

**Selection rule:** pick by the worst type first, then by the mean. Singer won.
