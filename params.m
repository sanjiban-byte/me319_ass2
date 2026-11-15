function p = params(cmd, ovr)
%PARAMS  Shared physical and design constants for the Guardian turret (ME319 A2).
%   p = params() returns a struct. This is the ONLY place constants are defined;
%   every other file reads them from here. Units: SI, angles in rad.
%
%   Design studies only (compare_designs.m): params('override', s) replaces
%   existing fields with those of struct s until params('override') clears it.
%   Unknown field names are an error. Never used by turret_controller itself.
%
%   Each part of the design has its own marked section below; add new fields in the
%   marked sections.

persistent P OVR
if nargin >= 1 && ischar(cmd) && strcmpi(cmd, 'override')
    if nargin < 2, OVR = []; else, OVR = ovr; end
    P = [];
    if nargout > 0, p = params(); end
    return
end
if ~isempty(P)
    p = P;
    return
end

%% ---------------- Problem data (assignment Section 2) ----------------
p.J_turret = 1200;          % turret + cradle inertia about trunnion [kg m^2]
p.J_m      = 0.20;          % motor rotor inertia [kg m^2]
p.N        = 100;           % gear ratio (motor speed / turret speed) [-]
p.b        = 1500;          % turret-side viscous damping [N m s/rad]
p.Kt       = 10;            % motor torque constant [N m/A]
p.Ke       = 10;            % motor back-emf constant [V s/rad]
p.R        = 1.0;           % armature resistance [ohm]
p.L        = 0.10;          % armature inductance [H]

p.theta_min = deg2rad(-10); % lower elevation stop [rad]
p.theta_max = deg2rad(85);  % upper elevation stop [rad]

p.dt_radar    = 0.05;           % radar sample interval (20 Hz) [s]
p.sigma_r     = 1.0;            % range noise std [m]
p.sigma_theta = deg2rad(0.02);  % elevation noise std [rad]
p.t_bo_max    = 0.50;           % max blackout duration [s]
p.t_boo_min   = 1.00;           % earliest blackout onset [s]

p.t_p        = 0.50;        % prescribed prediction horizon [s]
p.R_eng      = 2.0;         % engagement radius [m]
p.t_recharge = 1.0;         % minimum time between laser pulses [s]
p.t_end      = 10;          % standardized engagement duration [s]

p.r_min = 800;  p.r_max = 2000;   % operating range [m]
p.v_max = 150;  p.a_max = 20;     % target speed / acceleration limits

%% ---------------- Derived plant quantities ----------------
% Motor inertia reflected through the gearbox: J_m * N^2.
p.J_eq = p.J_turret + p.J_m * p.N^2;          % = 3200 kg m^2

% Turret-side torque per amp and back-emf per turret rad/s.
p.KtN = p.Kt * p.N;                            % = 1000 N m/A
p.KeN = p.Ke * p.N;                            % = 1000 V s/rad

% Linearized plant  Gp(s) = theta(s)/V(s) = KtN / ( s * [(J s + b)(L s + R) + KtN*KeN] )
p.Gp_num = p.KtN;                                          % 1000
p.Gp_den = [conv([p.J_eq p.b], [p.L p.R]) + [0 0 p.KtN*p.KeN], 0];
%        = [320  3350  1.0015e6  0]

% DC velocity gain theta_dot/V = KtN / (b R + KtN KeN) ~ 1/1001.5 rad/s per V
p.Kvel = p.KtN / (p.b*p.R + p.KtN*p.KeN);

% State-space form, x = [theta; theta_dot; i], u = V, y = theta
p.A = [0 1 0;
       0 -p.b/p.J_eq  p.KtN/p.J_eq;
       0 -p.KeN/p.L  -p.R/p.L];
p.B = [0; 0; 1/p.L];
p.C = [1 0 0];

%% ---------------- Actuator assumptions (decision D8) ----------------
p.V_max = 600;              % assumed drive voltage limit [V] (not given in spec)
p.tau_stop = 0.005;         % hard-stop velocity decay time constant [s] (plant model only)

%% ---------------- Feedback loop design (decision D18) ----------------
% Gc(s) = K_P * N(s), with N(s) a notch that cancels the plant's lightly damped
% electromechanical pole pair and replaces it with a well-damped pair at the
% same natural frequency:
%   N(s) = (s^2 + 2 zeta_p wn s + wn^2) / (s^2 + 2 zeta_d wn s + wn^2),  N(0) = 1
% K_P sets the loop velocity constant: Kv_fb = K_P * Kvel -> K_P = Kv_fb / Kvel.
% Implementation order (feedback_step.m): K_P -> saturation -> N -> clip, so the
% notch also smooths the voltage step at saturation onset. In the linear region
% this is exactly Gc(s) = K_P N(s).
p.wn_res   = sqrt(p.Gp_den(3) / p.Gp_den(1));                 % ~55.94 rad/s
p.zeta_res = p.Gp_den(2) / p.Gp_den(1) / (2*p.wn_res);        % ~0.0936
p.zeta_d   = 0.7;                   % notch target damping [-]
p.Kv_fb    = 8;                     % feedback-loop velocity constant [1/s]
p.K_P      = p.Kv_fb / p.Kvel;      % ~8012 V/rad
p.Gc_num = p.K_P * [1, 2*p.zeta_res*p.wn_res, p.wn_res^2];   % continuous, descending powers of s
p.Gc_den =         [1, 2*p.zeta_d  *p.wn_res, p.wn_res^2];

%% ---------------- Reference shaping and firing ----------------
% Overall reference->turret lag must equal t_p (decisions D2, D4).
p.Kv_total = 1 / p.t_p;             % = 2 1/s
p.tau_f    = p.t_p - 1/p.Kv_fb;     % first-order prefilter time constant [s] (all-pole form only)
p.prefilter_zero = true;    % D38: lead-lag prefilter with one zero (n = 1), matched so
                            % F*T*P = 1 up to s^2; removes the D33 residual.
                            % false = the original all-pole D23 filter.
p.n_pred  = 1;              % predictor order n (final decision: n = 1,
                            % theta_pred = theta_hat + t_p*theta_dot_hat). D2, D9.
                            % The shaper follows est.n from estimator_step anyway.
p.dt_ctrl = 0.05;           % internal update grid of turret_controller (D20) [s]
p.t_reset_jump = 0.5;       % backward time jump that counts as a new run (D19) [s]
p.reinit_on_reacq = true;   % FROZEN (D27): re-initialise the prefilter when measurements return after a blackout
p.t_warm  = 0.30;           % prefilter IC re-applied while the estimate settles [s]
% Firing gate (D39): valid AND |d_hat| < R_eng AND P_hit >= fire_phit_min
% AND recharge elapsed. See fire_logic.m. (The D28 rate and range checks are dropped.)
p.fire_phit_min = 0.95;     % P_hit threshold, refitted on Train-G (compare_designs, 115 cases; D39)
p.fire_recharge_margin = 0.01; % [s] edge spacing kept >= 1.01 s (float-safe; 1.05 s on the grid)
% Test bench (our scenarios only)
p.dt_sim     = 1e-3;        % plant integration step in the harness [s]
p.noise_seed = 2044;        % measurement-noise seed
p.theta0_sweep = deg2rad([-10 10 37.5 60 85]);   % D21

%% ---------------- Estimator and predictor (D30) ----------------
% estimator_step.m reads these (same names). Chosen on Train-E by the worst-case
% rule; see TUNABLE_PARAMETERS.md. CA was compared and not kept (comparison only).
p.est_model       = 'singer';  % Singer acceleration model (tuned)
p.est_sigma_m     = 10;        % Singer acceleration size per axis [m/s^2] (tuned)
p.est_tau_a       = 1.0;       % Singer acceleration fade time [s] (tuned)
p.est_sigma_v0    = p.v_max;   % speed uncertainty at the first reading [m/s] (set: envelope)
p.est_sigma_a0    = p.a_max;   % acceleration uncertainty at the first reading [m/s^2] (set: envelope)
p.est_pred_n      = 1;         % predictor order: theta_pred = theta_hat + t_p*theta_dot_hat
p.est_min_updates = 5;         % readings before est.valid can turn true (0.20 s)
p.est_valid_sigma = 1.0;       % sideways 1-sigma [m] needed for est.valid
p.est_coast_gap   = 1.5*p.dt_radar;  % no reading for this long -> est.coasting [s]

%% ---------------- Test settings (our scenarios only, decision D7) ----------------
p.theta0_test = deg2rad(37.5);      % turret initial angle in OUR test runs [rad]

%% ---------------- Study overrides (params('override', s)) ----------------
if ~isempty(OVR)
    f = fieldnames(OVR);
    for q = 1:numel(f)
        if ~isfield(p, f{q})
            error('params:badOverride', 'Unknown parameter "%s".', f{q});
        end
        p.(f{q}) = OVR.(f{q});
    end
end

P = p;
end
