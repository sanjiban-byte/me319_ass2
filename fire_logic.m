function [fire_cmd, info] = fire_logic(t, est, theta_turret, theta_dot_turret, reset)
%FIRE_LOGIC  Engagement gate and one-update 0->1 pulses.
%   Called on the D20 update grid. A pulse (fire_cmd = 1 for one update, so
%   the evaluator sees a single 0->1 edge) is requested when all hold (D39):
%     1. the estimator is valid (track settled);
%     2. the estimated miss is inside the engagement radius:
%           d = r_hat * (theta_turret - theta_hat),   |d| < R_eng;
%     3. the hit probability is high enough:
%           P_hit = Phi((R_eng - d)/sigma) - Phi((-R_eng - d)/sigma) >= fire_phit_min,
%        sigma = sigma_cross, Phi(x) = erfc(-x/sqrt(2))/2 (base MATLAB).
%        The true sideways offset is modelled as N(d, sigma^2); the laser is
%        instantaneous and theta_turret is known exactly at the firing instant.
%        sigma_cross grows in a blackout, so the gate closes by itself (D5);
%     4. t - t_last_pulse >= t_recharge + recharge margin (the margin keeps the
%        edge spacing above 1.0 s despite floating-point time stamps).
%   For fire_phit_min >= 0.5, condition 3 already implies condition 2; 2 is kept
%   as the plain, readable engagement test. The earlier turret-rate check
%   (|theta_dot_turret - theta_dot_hat| <= 0.02 rad/s, D28) and the elevation
%   range check were dropped (D39). theta_dot_turret is unused (kept for the
%   interface).
persistent t_last prev
p = params();
if reset || isempty(t_last), t_last = -Inf; prev = 0; end
fire_cmd = 0;
d = NaN; phit = 0; gate = false;
ready = (t - t_last) >= p.t_recharge + p.fire_recharge_margin;
if est.valid
    dth  = atan2(sin(theta_turret - est.theta_hat), cos(theta_turret - est.theta_hat));
    d    = est.r_hat*dth;                             % signed estimated miss [m]
    sg   = max(est.sigma_cross, 1e-9);
    Re   = p.R_eng;
    phit = 0.5*erfc(-(Re - d)/(sg*sqrt(2))) - 0.5*erfc(-(-Re - d)/(sg*sqrt(2)));
    gate = abs(d) < Re && phit >= p.fire_phit_min;
    if prev == 0 && ready && gate
        fire_cmd = 1;  t_last = t;
    end
end
prev = fire_cmd;                      % fire_cmd returns to 0 on the next update
info = struct('d_hat', d, 'phit', phit, 'gate', gate, 'ready', ready, 't_last', t_last);
end
