# Train-G study: P_hit threshold sweep (D39 gate), post-blackout reset on/off

115 cases: A, B, C at each theta0 (15), 50 random cases, 50 random cases with a forced full 0.5 s blackout.

## Prefilter reset after blackout (D26): ON

| gate | intercepted | first hit mean (s) | first hit worst (s) | pulses | missed | cases with a miss | mean miss (m) | p95 miss (m) | pulses in blackout (hits) |
|---|---|---|---|---|---|---|---|---|---|
| P_hit >= 0.5 | 115/115 | 0.94 | 2.55 | 1065 | 17 | 17 | 0.46 | 1.43 | 35 (34) |
| P_hit >= 0.8 | 115/115 | 0.87 | 2.55 | 1052 | 5 | 5 | 0.44 | 1.21 | 33 (32) |
| P_hit >= 0.9 | 115/115 | 0.85 | 2.55 | 1050 | 1 | 1 | 0.43 | 1.14 | 28 (27) |
| P_hit >= 0.95 | 115/115 | 0.86 | 2.55 | 1049 | 0 | 0 | 0.42 | 1.11 | 20 (20) |
| P_hit >= 0.977 | 115/115 | 0.86 | 2.55 | 1048 | 0 | 0 | 0.41 | 1.07 | 20 (20) |
| P_hit >= 0.99 | 115/115 | 0.86 | 2.55 | 1048 | 0 | 0 | 0.40 | 1.02 | 18 (18) |
| P_hit >= 0.995 | 115/115 | 0.87 | 2.55 | 1047 | 0 | 0 | 0.39 | 0.98 | 18 (18) |
| P_hit >= 0.999 | 115/115 | 0.88 | 2.60 | 1041 | 0 | 0 | 0.38 | 0.95 | 12 (12) |

After the blackout ends, cases locked before onset (88 of 93 with a blackout): peak |err| mean 1.34 m, worst 4.45 m; RMS mean 0.55 m; time to |err| <= 1 m (0.2 s held) mean 0.09 s, worst 0.37 s, never 0; blackout end -> next hit mean 0.38 s, worst 0.99 s.
Blackout during the first slew (not locked, 5 cases): blackout end -> next hit mean 0.32 s, worst 1.05 s.

sigma consistency (theta_hat error / sigma_cross; ideal 68 % / 95 % / 1.00): tracking 70 % / 96 % / RMS 0.95 (21994 samples); coasting 72 % / 98 % / RMS 0.88 (661 samples)

## Prefilter reset after blackout (D26): OFF

| gate | intercepted | first hit mean (s) | first hit worst (s) | pulses | missed | cases with a miss | mean miss (m) | p95 miss (m) | pulses in blackout (hits) |
|---|---|---|---|---|---|---|---|---|---|
| P_hit >= 0.5 | 115/115 | 0.94 | 2.55 | 1065 | 17 | 17 | 0.48 | 1.53 | 35 (34) |
| P_hit >= 0.8 | 115/115 | 0.87 | 2.55 | 1052 | 5 | 5 | 0.45 | 1.27 | 33 (32) |
| P_hit >= 0.9 | 115/115 | 0.85 | 2.55 | 1050 | 1 | 1 | 0.44 | 1.20 | 28 (27) |
| P_hit >= 0.95 | 115/115 | 0.86 | 2.55 | 1048 | 0 | 0 | 0.43 | 1.15 | 20 (20) |
| P_hit >= 0.977 | 115/115 | 0.86 | 2.55 | 1047 | 1 | 1 | 0.42 | 1.12 | 20 (20) |
| P_hit >= 0.99 | 115/115 | 0.86 | 2.55 | 1046 | 1 | 1 | 0.41 | 1.10 | 18 (18) |
| P_hit >= 0.995 | 115/115 | 0.87 | 2.55 | 1045 | 1 | 1 | 0.40 | 1.07 | 18 (18) |
| P_hit >= 0.999 | 115/115 | 0.88 | 2.60 | 1037 | 0 | 0 | 0.39 | 1.02 | 12 (12) |

After the blackout ends, cases locked before onset (88 of 93 with a blackout): peak |err| mean 1.39 m, worst 4.67 m; RMS mean 0.66 m; time to |err| <= 1 m (0.2 s held) mean 0.20 s, worst 1.12 s, never 0; blackout end -> next hit mean 0.40 s, worst 0.99 s.
Blackout during the first slew (not locked, 5 cases): blackout end -> next hit mean 0.33 s, worst 1.05 s.

sigma consistency (theta_hat error / sigma_cross; ideal 68 % / 95 % / 1.00): tracking 70 % / 96 % / RMS 0.95 (21994 samples); coasting 72 % / 98 % / RMS 0.88 (661 samples)

## Threshold selection (reset ON, worst-case rule)

Order: 1 worst-type success, 2 worst-run per-shot hit rate, 3 mean first hit, 4 total hits.

| Gate | Worst-type success | Worst-run hit rate | Mean first hit (s) | Total hits |
|---|---|---|---|---|
| P_hit >= 0.5 | 100.0 % | 88.9 % | 0.944 | 1048 |
| P_hit >= 0.8 | 100.0 % | 88.9 % | 0.870 | 1047 |
| P_hit >= 0.9 | 100.0 % | 88.9 % | 0.845 | 1049 |
| P_hit >= 0.95 **(selected)** | 100.0 % | 100.0 % | 0.856 | 1049 |
| P_hit >= 0.977 | 100.0 % | 100.0 % | 0.858 | 1048 |
| P_hit >= 0.99 | 100.0 % | 100.0 % | 0.862 | 1048 |
| P_hit >= 0.995 | 100.0 % | 100.0 % | 0.868 | 1047 |
| P_hit >= 0.999 | 100.0 % | 100.0 % | 0.880 | 1041 |
