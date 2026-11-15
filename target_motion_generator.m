function [r_meas, theta_meas, meas_valid] = target_motion_generator(t, scenario)
%TARGET_MOTION_GENERATOR  Radar measurements of the V-44 target (test version).
%   [r_meas, theta_meas, meas_valid] = target_motion_generator(t, scenario)
%   t          time [s]
%   scenario   'A' | 'B' | 'C' (or 1|2|3); integer >= 4 = seeded random case
%   r_meas     measured range [m]
%   theta_meas measured elevation angle [rad]
%   meas_valid true only at a 20 Hz radar instant (t = k*0.05 s) that is not
%              inside a blackout window [start, end) (S.bo, one or more)
%   Noise: zero-mean white Gaussian, sigma_r = 1 m, sigma_theta = 0.02 deg,
%   one independent draw per radar sample, precomputed per scenario in
%   target_scenario (seeded; the global RNG is left untouched). It is a pure
%   function of (sample index, scenario), so runs are reproducible. When
%   meas_valid is false the returned r_meas and theta_meas are NaN.
p  = params();
S  = target_scenario(scenario);
k  = round(t/p.dt_radar);
on_sample = abs(t - k*p.dt_radar) < 1e-6 && t > -1e-9;
in_blackout = any(t >= S.bo(:,1) - 1e-9 & t < S.bo(:,2) - 1e-9);   % any number of windows
meas_valid = on_sample && ~in_blackout;
if ~meas_valid
    r_meas = NaN; theta_meas = NaN; return;
end
[~, ~, r, theta] = target_truth(k*p.dt_radar, scenario);
r_meas     = r     + p.sigma_r     * S.noise(k+1, 1);
theta_meas = theta + p.sigma_theta * S.noise(k+1, 2);
end
