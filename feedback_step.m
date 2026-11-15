function V_m = feedback_step(theta_ref, theta_turret, theta_dot_turret, dt)
%FEEDBACK_STEP  Turret elevation feedback controller Gc(s) with voltage limit.
%   V_m = feedback_step(theta_ref, theta_turret, theta_dot_turret, dt)
%   feedback_step('reset')          % clear internal state (call at t = 0)
%
%   Inputs : theta_ref       shaped elevation reference from reference_shaper [rad]
%            theta_turret    measured turret elevation [rad]
%            theta_dot_turret measured turret rate [rad/s] (unused by this design)
%            dt              time since the previous call [s]
%   Output : V_m motor voltage command [V], |V_m| <= p.V_max.
%
%   Design (decision D18, see design_controller.m):
%       Gc(s) = K_P * N(s),   K_P = Kv_fb / Kvel,
%       N(s)  = (s^2 + 2 zeta_res wn s + wn^2) / (s^2 + 2 zeta_d wn s + wn^2).
%   N cancels the plant's lightly damped pole pair (wn ~ 56 rad/s, zeta ~ 0.094)
%   and replaces it with zeta_d = 0.7. The loop is type-1 with velocity
%   constant Kv_fb; no integral action (D3).
%
%   Signal order:  e -> K_P -> saturate(+/-V_max) -> N(z) -> clip -> V_m.
%   Putting N after the saturation means the voltage jump at saturation onset
%   is smoothed too, so it does not ring the 56 rad/s mode. When not saturated
%   the cascade is exactly Gc. |N(jw)| <= 1 at all frequencies; the final clip
%   only guards against small transient overshoot.
%
%   Discretization: N is discretized by pole-zero matching (z = e^{sT}) at the
%   actual dt, so its zeros land exactly on the ZOH-sampled plant poles at any
%   call rate (a Tustin notch does not, and goes unstable at dt = 50 ms).
%   Coefficients are recomputed whenever dt changes; DC gain is 1.
%
%   Call-time robustness (D12, D18): calls with dt <= 0 (repeated or earlier
%   times, e.g. ODE solver stages) return the previous output without updating
%   the state. The caller (turret_controller.m) owns reset detection.

persistent u1 u2 y1 y2 V_last dt_c b0 b1 b2 a1 a2

if ischar(theta_ref) && strcmpi(theta_ref, 'reset')
    u1 = 0; u2 = 0; y1 = 0; y2 = 0; V_last = 0; dt_c = [];
    V_m = 0;
    return
end
if isempty(V_last)                      % first call without an explicit reset
    u1 = 0; u2 = 0; y1 = 0; y2 = 0; V_last = 0; dt_c = [];
end
if dt <= 0
    V_m = V_last;
    return
end

p = params();

% ---- (Re)compute matched notch coefficients for this dt ----
if isempty(dt_c) || abs(dt - dt_c) > 1e-12
    [nz, dz] = matched_pair(p.wn_res, p.zeta_res, p.zeta_d, dt);
    k  = sum(dz) / sum(nz);             % unity DC gain: N(z=1) = 1
    b0 = k*nz(1); b1 = k*nz(2); b2 = k*nz(3);
    a1 = dz(2);   a2 = dz(3);
    dt_c = dt;
end

% ---- Proportional gain and actuator limit ----
u = p.K_P * (theta_ref - theta_turret);
u = min(max(u, -p.V_max), p.V_max);

% ---- Notch (direct form I) ----
y = b0*u + b1*u1 + b2*u2 - a1*y1 - a2*y2;
u2 = u1; u1 = u;
y2 = y1; y1 = y;

V_m = min(max(y, -p.V_max), p.V_max);
V_last = V_m;
end

function [nz, dz] = matched_pair(wn, zeta_num, zeta_den, T)
% z-domain monic quadratics whose roots are e^{sT} of the s-domain roots of
% s^2 + 2 zeta wn s + wn^2 (numerator: zeta_num, denominator: zeta_den).
nz = quad_z(wn, zeta_num, T);
dz = quad_z(wn, zeta_den, T);
end

function q = quad_z(wn, zeta, T)
if zeta < 1
    wd = wn*sqrt(1 - zeta^2);
    r  = exp(-zeta*wn*T);
    q  = [1, -2*r*cos(wd*T), r^2];
else
    s12 = -zeta*wn + [1 -1]*wn*sqrt(zeta^2 - 1);
    zr  = exp(s12*T);
    q   = [1, -sum(zr), prod(zr)];
end
end
