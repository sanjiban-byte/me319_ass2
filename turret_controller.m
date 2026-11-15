function [V_m, fire_cmd] = turret_controller(t, r_meas, theta_meas, meas_valid, theta_turret, theta_dot_turret)
%TURRET_CONTROLLER  Predictive-feedback fire-control wrapper (owner: Harsh).
%   [V_m, fire_cmd] = turret_controller(t, r_meas, theta_meas, meas_valid, ...
%                                       theta_turret, theta_dot_turret)
%   t [s]; r_meas [m]; theta_meas [rad]; meas_valid (new radar sample?);
%   theta_turret [rad]; theta_dot_turret [rad/s].
%   V_m motor voltage [V] (saturated at +/- V_max, D8); fire_cmd binary,
%   a pulse is a 0->1 transition.
%
%   Pipeline: estimator_step -> reference_shaper -> feedback_step, with
%   fire_logic alongside.
%   D20 (update grid): a valid measurement is passed to the estimator on
%   whatever call delivers it. The shaper, feedback_step and fire_logic update
%   only on the fixed grid t_k = k*dt_ctrl (0.05 s, aligned with the radar);
%   between grid instants V_m and fire_cmd are held. If calls are sparser than
%   the grid, every call is an update with its actual dt.
%   D19 (call times): reset on the first call or when t jumps back by more
%   than t_reset_jump (a new run). A repeated or slightly earlier t returns
%   the held outputs and updates nothing except ingesting a radar sample that
%   has not been seen before (safe inside an ODE solver).
persistent t_last t_upd k_next V_hold F_hold fresh was_coasting t_meas
p = params();
dtc = p.dt_ctrl;
if isempty(t_last) || t < t_last - p.t_reset_jump               % new run
    clear estimator_step reference_shaper fire_logic
    feedback_step('reset');
    t_last = -Inf; t_upd = []; k_next = ceil(t/dtc - 1e-9);
    V_hold = 0; F_hold = 0; fresh = true; was_coasting = false; t_meas = -Inf;
end
est = [];
if meas_valid && t > t_meas + 1e-9                               % new radar sample: ingest
    est = estimator_step(t, r_meas, theta_meas, true);           % on any call (D20)
    t_meas = t;
end
if t <= t_last                                                   % repeat / small step back:
    V_m = V_hold; fire_cmd = F_hold; return                      % hold outputs
end
t_last = t;

if t >= k_next*dtc - 1e-9                                        % grid update
    if isempty(est), est = estimator_step(t, NaN, NaN, false); end
    if isempty(t_upd), dt = dtc; else, dt = t - t_upd; end
    if abs(dt - dtc) < 1e-6, dt = dtc; end                      % snap float jitter to the grid
    reset_blk = fresh;
    if est.valid
        reacq = was_coasting && ~est.coasting && p.reinit_on_reacq;   % blackout just ended
        [theta_ref, sh] = reference_shaper(est, dt, fresh || reacq);  fresh = false;
        was_coasting = est.coasting;
        V_hold = feedback_step(theta_ref, theta_turret, theta_dot_turret, dt);
        V_hold = min(max(V_hold, -p.V_max), p.V_max);
        dbg_ref = [theta_ref, sh.theta_ref_raw];
    else                                                         % no track yet: hold still
        V_hold = 0;  dbg_ref = [NaN, NaN];
    end
    [F_hold, fi] = fire_logic(t, est, theta_turret, theta_dot_turret, reset_blk);
    t_upd = t;  k_next = floor(t/dtc + 1e-9) + 1;
    turret_debug('set', struct('est', est, 'theta_ref', dbg_ref(1), ...
                 'theta_ref_raw', dbg_ref(2), 'fire', fi, 'updated', true));
end
V_m = V_hold;  fire_cmd = F_hold;
end
