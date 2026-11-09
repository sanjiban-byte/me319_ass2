# Estimator test: confirm_seed2
40 runs per drone type, call every 5 ms, radar style "pulse", seed 2. Same drones and radar noise for every setting.

## 1. Ideal matched turret: % of time on target (miss <= 2 m), mean over runs
Scored from the moment est.valid turns true until t = 10 s. "worst type" = lowest drone-type mean; "worst run" = lowest single run.

| setting | straight | w_slow | w_med | w_fast | jink | random | near_fast | far | bo_C | bo_jink | bo_random | bo_double | mean | worst type | worst run | p99 miss (m) | t valid (s) |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| CA q=100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 99 | 99 | 99 | 98 | 99.5 | 97.9 | 88 | 2.75 | 0.20 |
| CA q=250 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 99 | 99 | 99 | 98 | 99.6 | 98.0 | 92 | 2.72 | 0.20 |
| CA q=500 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 99 | 98 | 99 | 98 | 99.5 | 97.6 | 91 | 2.84 | 0.20 |
| Singer tau=0.5 sig=7.5 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 99 | 99 | 98 | 99.6 | 97.6 | 88 | 2.46 | 0.20 |
| Singer tau=0.5 sig=10 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 99 | 99 | 98 | 99.7 | 97.9 | 90 | 2.39 | 0.20 |
| Singer tau=1 sig=10 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 99 | 99 | 98 | 99.7 | 98.2 | 91 | 2.42 | 0.20 |

## 2. During blackout + 0.5 s recovery: % of time on target (ideal turret)

| setting | bo_C | bo_jink | bo_random | bo_double |
|---|---|---|---|---|
| CA q=100 | 94 | 91 | 85 | 90 |
| CA q=250 | 93 | 88 | 87 | 90 |
| CA q=500 | 92 | 86 | 87 | 89 |
| Singer tau=0.5 sig=7.5 | 99 | 91 | 89 | 89 |
| Singer tau=0.5 sig=10 | 99 | 91 | 91 | 90 |
| Singer tau=1 sig=10 | 98 | 91 | 90 | 91 |

## 3. Current estimate theta_hat: % of time within 2 m / RMS sideways error (m)

| setting | straight | w_slow | w_med | w_fast | jink | random | near_fast | far | bo_C | bo_jink | bo_random | bo_double |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| CA q=100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 99 | 99 |
| CA q=250 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 99 | 99 |
| CA q=500 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 99 | 99 | 99 |
| Singer tau=0.5 sig=7.5 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 99 |
| Singer tau=0.5 sig=10 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 99 |
| Singer tau=1 sig=10 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 100 | 99 |

RMS sideways error of theta_hat (m):

| setting | straight | w_slow | w_med | w_fast | jink | random | near_fast | far | bo_C | bo_jink | bo_random | bo_double |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| CA q=100 | 0.29 | 0.31 | 0.32 | 0.35 | 0.32 | 0.32 | 0.28 | 0.37 | 0.39 | 0.37 | 0.42 | 0.47 |
| CA q=250 | 0.31 | 0.33 | 0.33 | 0.35 | 0.34 | 0.33 | 0.30 | 0.39 | 0.41 | 0.41 | 0.42 | 0.49 |
| CA q=500 | 0.33 | 0.35 | 0.35 | 0.36 | 0.36 | 0.34 | 0.31 | 0.40 | 0.43 | 0.45 | 0.44 | 0.52 |
| Singer tau=0.5 sig=7.5 | 0.28 | 0.31 | 0.32 | 0.33 | 0.31 | 0.31 | 0.27 | 0.37 | 0.36 | 0.34 | 0.39 | 0.45 |
| Singer tau=0.5 sig=10 | 0.30 | 0.32 | 0.32 | 0.33 | 0.32 | 0.31 | 0.28 | 0.37 | 0.37 | 0.37 | 0.39 | 0.46 |
| Singer tau=1 sig=10 | 0.29 | 0.31 | 0.32 | 0.33 | 0.32 | 0.31 | 0.28 | 0.37 | 0.37 | 0.36 | 0.39 | 0.45 |

## 4. Pure 0.5 s forecast (info only): % of time theta_pred is within 2 m of the drone 0.5 s later

| setting | straight | w_slow | w_med | w_fast | jink | random | near_fast | far | bo_C | bo_jink | bo_random | bo_double |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| CA q=100 | 93 | 82 | 74 | 66 | 87 | 79 | 65 | 75 | 75 | 86 | 72 | 74 |
| CA q=250 | 90 | 80 | 75 | 69 | 84 | 79 | 64 | 75 | 75 | 83 | 73 | 74 |
| CA q=500 | 86 | 77 | 73 | 70 | 81 | 77 | 63 | 73 | 73 | 80 | 73 | 72 |
| Singer tau=0.5 sig=7.5 | 95 | 74 | 70 | 70 | 89 | 77 | 66 | 70 | 69 | 88 | 69 | 71 |
| Singer tau=0.5 sig=10 | 93 | 74 | 71 | 72 | 87 | 78 | 65 | 71 | 71 | 86 | 71 | 72 |
| Singer tau=1 sig=10 | 94 | 77 | 73 | 70 | 88 | 78 | 65 | 73 | 73 | 87 | 72 | 73 |

## 5. Worst-case rule

- Best CA (by worst type): **CA q=250**: mean 99.6, worst type 98.0 (blackout_double), worst run 92
- Best Singer (by worst type): **Singer tau=1 sig=10**: mean 99.7, worst type 98.2 (blackout_double), worst run 91
- CA minus Singer, per run: -0.10 points (2 x std. error = 0.06)
- CA average not much worse: no. CA worst type better: no.
- **Rule says: singer** (to be confirmed by the team, and on a fresh seed)

## 6. OLS fit theta_hat = a + b * theta_true (best CA and best Singer)
a in micro-rad, b = slope, R2. A perfect estimator gives a = 0, b = 1, R2 = 1.

**CA q=250**

| type | a (urad) | b | R2 |
|---|---|---|---|
| straight | -1.0 | 1.000002 | 0.99999943 |
| w_slow | -10.1 | 1.000007 | 0.99999960 |
| w_med | 2.4 | 0.999999 | 0.99999952 |
| w_fast | 9.5 | 0.999993 | 0.99999948 |
| jink | -0.8 | 1.000003 | 0.99999952 |
| random | -5.5 | 1.000003 | 0.99999952 |
| near_fast | -19.4 | 1.000028 | 0.99999946 |
| far | 10.0 | 0.999989 | 0.99999963 |
| bo_C | 2.7 | 1.000001 | 0.99999947 |
| bo_jink | -18.0 | 1.000017 | 0.99999922 |
| bo_random | -15.9 | 1.000005 | 0.99999926 |
| bo_double | -9.5 | 0.999985 | 0.99999898 |

**Singer tau=1 sig=10**

| type | a (urad) | b | R2 |
|---|---|---|---|
| straight | -1.3 | 1.000002 | 0.99999950 |
| w_slow | -13.2 | 1.000015 | 0.99999964 |
| w_med | -0.6 | 1.000005 | 0.99999956 |
| w_fast | 9.7 | 0.999994 | 0.99999952 |
| jink | 3.9 | 0.999998 | 0.99999958 |
| random | -4.4 | 1.000002 | 0.99999957 |
| near_fast | -18.6 | 1.000028 | 0.99999953 |
| far | 7.4 | 0.999993 | 0.99999967 |
| bo_C | -0.7 | 1.000004 | 0.99999957 |
| bo_jink | -15.9 | 1.000018 | 0.99999939 |
| bo_random | -14.0 | 0.999999 | 0.99999937 |
| bo_double | -8.3 | 0.999995 | 0.99999914 |

