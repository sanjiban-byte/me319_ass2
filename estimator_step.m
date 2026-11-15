function est = estimator_step(t, r_meas, theta_meas, meas_valid)
%ESTIMATOR_STEP  Radar denoiser and 0.5 s predictor for the Guardian turret (ME319 A2).
%
%   est = estimator_step(t, r_meas, theta_meas, meas_valid)
%
%   Call once per controller call. Keeps its own state between calls (persistent)
%   and resets automatically when t jumps back by more than 0.5 s (a new run,
%   decision D19). Smaller backward steps, as an ODE solver makes on rejected
%   steps, keep the track: the state is just not moved backwards.
%
%   Inputs
%     t           current time [s]
%     r_meas      radar range [m]           (ignored unless this call carries a new reading)
%     theta_meas  radar elevation [rad]     (ignored unless this call carries a new reading)
%     meas_valid  1 = the radar reading in this call is real, 0 = no reading
%                 (blackout, or between the 20 Hz radar samples)
%
%   A reading is used only if meas_valid is 1 AND (r_meas, theta_meas) differ from the
%   last reading we used. So it works whether the caller sends meas_valid = 1 only at
%   the radar instants, or keeps repeating the last reading with meas_valid = 1.
%
%   Output struct est (all angles in rad, SI units; interface of the work-split doc, 5.2)
%     theta_hat       current elevation estimate, brought up to time t (D16)
%     r_hat           current range estimate [m]
%     theta_dot_hat   elevation rate [rad/s]
%     theta_ddot_hat  elevation acceleration [rad/s^2]
%     theta_pred      reference for the turret: theta_hat + t_p*theta_dot_hat (n = 1),
%                     plus c2*theta_ddot_hat if n = 2. NOT clamped (the clamp is in reference_shaper, D6).
%     n               predictor order used (1 or 2)
%     sigma_cross     1-sigma sideways (cross-beam) position uncertainty now [m]
%     coasting        true while no radar reading has arrived for longer than est_coast_gap
%     valid           true once the track has settled (at least est_min_updates readings
%                     and sigma_cross <= est_valid_sigma). Latched: stays true until reset.
%     t_since_meas    time since the last reading used [s]
%     n_updates       number of readings used so far
%     state           filter state [x vx ax y vy ay] (turret at origin, y up) for plots
%     pos_pred        Cartesian forecast of the drone at t + t_p [x; y] (plots only)
%   Before the first reading every number is NaN and valid = false.
%
%   Control calls
%     estimator_step('config', s)  set tunable values for the next runs (struct with
%                                  est_* fields; unknown names are an error). Resets.
%     estimator_step('config')     back to the default values. Resets.
%     cfg = estimator_step('getconfig')   the values in use
%     estimator_step('reset')      clear the track (start of a new run)
%
%   Tunable values (see TUNABLE_PARAMETERS.md). Defaults are used unless params.m has
%   an est_* field of the same name or 'config' overrides it.
%
%   Method: Kalman filter in (x, y). Each radar reading (r, theta) is converted to (x, y);
%   its noise is mostly sideways at long range, so the measurement noise is the rotated
%   diag(sigma_r^2, (r*sigma_theta)^2). Motion model per axis [p v a]:
%     'CA'     constant acceleration with random jerk (intensity est_q_ca). NOT part of the
%              final design: kept only so estimator_tests/run_estimator_sweeps.m can
%              reproduce the CA-vs-Singer comparison that led to choosing Singer.
%     'singer' acceleration that fades with time constant est_tau_a, typical size est_sigma_m
%   Both use exact continuous-time noise, so any call rate works.
%
%   ME319 Assignment 2.

persistent S CFG OVR

% ---------------- control calls ----------------
if ischar(t) || isstring(t)
    est = [];
    switch lower(char(t))
        case 'config'
            if nargin < 2 || isempty(r_meas)
                OVR = struct();
            else
                OVR = r_meas;
            end
            CFG = build_config(OVR);
            S = [];
        case 'getconfig'
            if isempty(CFG), CFG = build_config(OVR); end
            est = CFG;
        case 'reset'
            S = [];
        otherwise
            error('estimator_step:badCommand', 'Unknown command "%s".', char(t));
    end
    return
end

if isempty(CFG), CFG = build_config(OVR); end

% ---------------- new run? (D19: reset only on a big jump back in time) ----------------
% An ODE solver may call slightly earlier times (rejected steps). Those are not a
% new run: the state is simply not moved backwards. Only a jump back by more than
% t_reset_jump (0.5 s) starts a new track.
if isempty(S) || t < S.t_call - CFG.t_reset_jump
    S = new_track(t);
end
S.t_call = max(S.t_call, t);

% ---------------- bring the state up to time t (D16) ----------------
if S.have
    dt = t - S.t_state;
    if dt > 0
        [F, Q, S] = transition(dt, CFG, S);
        S.x = F * S.x;
        S.P = F * S.P * F' + Q;
        S.P = (S.P + S.P') / 2;
    end
end
S.t_state = max(S.t_state, t);           % never moves backwards

% ---------------- use the radar reading if it is new ----------------
is_new = ~isempty(meas_valid) && logical(meas_valid(1)) ...
    && isfinite(r_meas) && isfinite(theta_meas) && r_meas > 0 ...
    && ~(r_meas == S.last_r && theta_meas == S.last_th);

if is_new
    c = cos(theta_meas); s = sin(theta_meas);
    z = r_meas * [c; s];
    Rot = [c -s; s c];
    R = Rot * diag([CFG.sigma_r^2, (r_meas * CFG.sigma_theta)^2]) * Rot';
    if ~S.have
        % First reading: position from the radar, speed and acceleration unknown.
        S.x = zeros(6, 1);
        S.x([1 4]) = z;
        S.P = zeros(6);
        S.P([1 4], [1 4]) = R;
        S.P(2, 2) = CFG.est_sigma_v0^2;  S.P(5, 5) = CFG.est_sigma_v0^2;
        S.P(3, 3) = CFG.est_sigma_a0^2;  S.P(6, 6) = CFG.est_sigma_a0^2;
        S.have = true;
    else
        H = zeros(2, 6); H(1, 1) = 1; H(2, 4) = 1;
        innov = z - H * S.x;
        Sm = H * S.P * H' + R;
        K = (S.P * H') / Sm;
        S.x = S.x + K * innov;
        IKH = eye(6) - K * H;
        S.P = IKH * S.P * IKH' + K * R * K';      % Joseph form: stays symmetric, positive
        S.P = (S.P + S.P') / 2;
    end
    S.n_upd = S.n_upd + 1;
    S.t_meas = max(S.t_meas, t);
    S.last_r = r_meas;
    S.last_th = theta_meas;
end

% ---------------- outputs ----------------
est = make_output(S, CFG, t);
S.valid = est.valid;                      % latch
end


% =====================================================================
function S = new_track(t)
S.have = false;
S.t_call = t;
S.t_state = t;
S.x = nan(6, 1);
S.P = nan(6);
S.n_upd = 0;
S.t_meas = -inf;
S.last_r = NaN;
S.last_th = NaN;
S.valid = false;
S.cache_dt = NaN;
S.cache_F = [];
S.cache_Q = [];
end


% =====================================================================
function est = make_output(S, CFG, t)
est.theta_hat = NaN;
est.r_hat = NaN;
est.theta_dot_hat = NaN;
est.theta_ddot_hat = NaN;
est.theta_pred = NaN;
est.n = CFG.est_pred_n;
est.sigma_cross = NaN;
est.coasting = true;
est.valid = false;
est.t_since_meas = t - S.t_meas;
est.n_updates = S.n_upd;
est.state = S.x;
est.pos_pred = [NaN; NaN];
if ~S.have
    return
end

px = S.x(1); vx = S.x(2); ax = S.x(3);
py = S.x(4); vy = S.x(5); ay = S.x(6);
r2 = px^2 + py^2;
r = sqrt(r2);
th = atan2(py, px);
thd = (px * vy - py * vx) / r2;
rd = (px * vx + py * vy) / r;
thdd = (px * ay - py * ax) / r2 - 2 * rd * thd / r;

u = [-sin(th); cos(th)];                 % sideways (cross-beam) direction
Pp = S.P([1 4], [1 4]);
sig = sqrt(max(u' * Pp * u, 0));

if CFG.est_pred_n == 2
    th_pred = th + CFG.t_p * thd + CFG.est_pred_c2 * thdd;
else
    th_pred = th + CFG.t_p * thd;
end

est.theta_hat = th;
est.r_hat = r;
est.theta_dot_hat = thd;
est.theta_ddot_hat = thdd;
est.theta_pred = th_pred;
est.sigma_cross = sig;
est.coasting = (t - S.t_meas) > CFG.est_coast_gap;
est.valid = S.valid || (S.n_upd >= CFG.est_min_updates && sig <= CFG.est_valid_sigma);
Fp = blkdiag(CFG.Fp3, CFG.Fp3);
xp = Fp * S.x;
est.pos_pred = xp([1 4]);
end


% =====================================================================
function [F, Q, S] = transition(dt, CFG, S)
% Exact discrete model over dt (cached, since dt is usually the same every call).
if abs(dt - S.cache_dt) <= 1e-12
    F = S.cache_F; Q = S.cache_Q;
    return
end
[F3, Q3] = axis_model(dt, CFG);
F = blkdiag(F3, F3);
Q = blkdiag(Q3, Q3);
S.cache_dt = dt; S.cache_F = F; S.cache_Q = Q;
end


function [F3, Q3] = axis_model(T, CFG)
switch CFG.est_model
    case 'CA'
        q = CFG.est_q_ca;
        F3 = [1 T T^2/2; 0 1 T; 0 0 1];
        Q3 = q * [T^5/20 T^4/8 T^3/6;
                  T^4/8  T^3/3 T^2/2;
                  T^3/6  T^2/2 T];
    case 'singer'
        tau = CFG.est_tau_a;
        qc = 2 * CFG.est_sigma_m^2 / tau;
        A = [0 1 0; 0 0 1; 0 0 -1/tau];
        G = [0; 0; 1];
        M = [-A, G * G' * qc; zeros(3), A'];     % Van Loan
        E = expm(M * T);
        F3 = E(4:6, 4:6)';
        Q3 = F3 * E(1:3, 4:6);
        Q3 = (Q3 + Q3') / 2;
end
end


% =====================================================================
function CFG = build_config(OVR)
p = params();
CFG.sigma_r = p.sigma_r;
CFG.sigma_theta = p.sigma_theta;
CFG.t_p = p.t_p;
CFG.dt_radar = p.dt_radar;
if isfield(p, 't_reset_jump')
    CFG.t_reset_jump = p.t_reset_jump;   % same rule as turret_controller (D19)
else
    CFG.t_reset_jump = 0.5;
end

% Defaults (Candidate values; frozen values move to params.m).
D.est_model = 'singer';      % final design: Singer (chosen over CA by the sweep, worst-case rule)
D.est_q_ca = 250;            % CA jerk intensity [m^2/s^5]; comparison runs only, not used
D.est_sigma_m = 10;          % Singer typical acceleration per axis [m/s^2]
D.est_tau_a = 1.0;           % Singer acceleration fade time [s]
D.est_sigma_v0 = p.v_max;    % speed uncertainty at the first reading [m/s]
D.est_sigma_a0 = p.a_max;    % acceleration uncertainty at the first reading [m/s^2]
D.est_pred_n = 1;            % predictor order (decided: 1; 2 kept for the fallback test)
D.est_pred_c2 = p.t_p^2 / 2; % second-order coefficient, used only if est_pred_n = 2
D.est_min_updates = 5;       % readings needed before est.valid can turn true
D.est_valid_sigma = 1.0;     % sideways uncertainty [m] needed before est.valid
D.est_coast_gap = 1.5 * p.dt_radar;  % no reading for this long -> est.coasting

names = fieldnames(D);
for i = 1:numel(names)
    nm = names{i};
    if isfield(p, nm)
        D.(nm) = p.(nm);         % frozen value from params.m
    end
end
if ~isempty(OVR)
    on = fieldnames(OVR);
    for i = 1:numel(on)
        if ~isfield(D, on{i})
            error('estimator_step:badConfig', 'Unknown tunable "%s".', on{i});
        end
        D.(on{i}) = OVR.(on{i});
    end
end

if strcmpi(D.est_model, 'singer')
    D.est_model = 'singer';
elseif strcmpi(D.est_model, 'CA')
    D.est_model = 'CA';
else
    error('estimator_step:badConfig', 'est_model must be ''CA'' or ''singer''.');
end
if ~ismember(D.est_pred_n, [1 2])
    error('estimator_step:badConfig', 'est_pred_n must be 1 or 2.');
end

for i = 1:numel(names)
    CFG.(names{i}) = D.(names{i});
end
[CFG.Fp3, ~] = axis_model(CFG.t_p, CFG);  % for the Cartesian forecast (plots)
end
