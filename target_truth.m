function [x, y, r, theta, vx, vy, ax, ay] = target_truth(t, scenario)
%TARGET_TRUTH  True target state at time(s) t for the given scenario.
%   Units: m, m/s, m/s^2, rad. x = downrange, y = up, theta from horizontal.
%   t may be a vector. Used for scoring hits (never by the controller).
S  = target_scenario(scenario);
tt = min(max(t, 0), S.t(end));
x  = interp1(S.t, S.x,  tt, 'linear');  y  = interp1(S.t, S.y,  tt, 'linear');
vx = interp1(S.t, S.vx, tt, 'linear');  vy = interp1(S.t, S.vy, tt, 'linear');
ax = interp1(S.t, S.ax, tt, 'linear');  ay = interp1(S.t, S.ay, tt, 'linear');
r  = hypot(x, y);
theta = atan2(y, x);
end
