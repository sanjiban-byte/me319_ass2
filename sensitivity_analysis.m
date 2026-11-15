%SENSITIVITY_ANALYSIS  Turret-side sensitivity results for the paper (step 5, T1).
%   A1  Measurement/prediction noise -> turret jitter and voltage jitter
%   A2  Prediction error -> turret: frequency response of F(z)T(z), total lag
%   A3  Blackout-recovery reference step: worst-case size and turret recovery
%   A4  Large-slew acquisition (voltage saturation) vs slew size
%   B   Voltage and current needed for tracking (analytic bounds)
%   All at the 50 ms internal update grid (decision D20). Requires the
%   Control System Toolbox. Figures are saved to ./figures.

clear params feedback_step
p   = params();
T   = p.dt_radar;                         % 50 ms grid (D20)
d2r = pi/180;
figdir = fullfile(fileparts(mfilename('fullpath')), 'figures');
if ~exist(figdir, 'dir'), mkdir(figdir); end

%% ---- Discrete models on the 50 ms grid ----
s  = tf('s');
Gp = tf(p.Gp_num, p.Gp_den);
N  = (s^2 + 2*p.zeta_res*p.wn_res*s + p.wn_res^2) / (s^2 + 2*p.zeta_d*p.wn_res*s + p.wn_res^2);
Gd = c2d(Gp, T, 'zoh');                   % plant seen at the sample instants
Cd = p.K_P * c2d(N, T, 'matched');        % same discretization as feedback_step.m
Dpf = prefilter_design(p.n_pred, T);     % the prefilter actually used (D38 lead-lag if p.prefilter_zero)
Fd = tf([Dpf.num, zeros(1, numel(Dpf.den) - numel(Dpf.num))], Dpf.den, T);   % F(z), same powers of z

Td  = feedback(Cd*Gd, 1);                 % reference -> turret (feedback loop)
Sd  = feedback(1, Cd*Gd);                 % sensitivity
FT  = Fd * Td;                            % shaped reference -> turret
FCS = Fd * Cd * Sd;                       % shaped reference -> voltage

%% ---- A1: noise gains ----
g_th = h2_gain(FT);                       % turret rms per unit white reference noise
g_V  = h2_gain(FCS);                      % volts rms per rad of white reference noise
sig_raw = p.sigma_theta;                  % placeholder: raw radar angle noise
fprintf('A1  White-noise gains (50 ms grid):\n');
fprintf('    turret: %.3f rad/rad  -> with raw radar noise (%.3f mrad): %.3f mrad = %.2f m at 2 km\n', ...
    g_th, 1e3*sig_raw, 1e3*g_th*sig_raw, 2000*g_th*sig_raw);
fprintf('    voltage: %.0f V/rad    -> %.1f V rms\n', g_V, g_V*sig_raw);
budget = 0.5;                             % allowed turret jitter at 2 km [m]
fprintf('    Spec for estimator: white sigma(theta_pred) <= %.2f mrad keeps jitter <= %.1f m at 2 km\n', ...
    1e3*budget/2000/g_th, budget);
fprintf('    (slowly varying errors are not attenuated; see A2)\n\n');

%% ---- A1b: closed-loop tracking-error budget (estimator_step, D30) ----
% Scenarios A, B, C at the 37.5 deg baseline, t >= 2.5 s (turret locked).
% Linear model: turret ~= FT*theta_pred with theta_pred = (theta + t_p*thetadot) + e_pred:
%   turret - theta = FT*e_pred                              (estimator part)
%                  + [FT*(theta + t_p*thetadot) - theta]    (lag-mismatch residual; D33,
%                                                             about 0 with the D38 zero)
% The residual is not zero even for a straight-line target: the line of sight
% still accelerates (r*thetaddot ~ -2*rdot*thetadot). Both parts are computed by
% filtering the logged 50 ms signals through F(z)T(z) from t = 0 (so filter
% start-up transients have died out by 2.5 s) and compared with the turret
% error measured in the closed-loop simulation.
[nF, dF] = tfdata(FT, 'v');
nF = [zeros(1, numel(dF) - numel(nF)), nF];       % same length as dF: no time shift in filter()
fz = @(u) filter(nF, dF, u - u(1)) + u(1);        % FT with a steady start (DC gain 1)
fprintf('A1b Closed-loop tracking-error budget (t >= 2.5 s, RMS cross-range metres):\n');
fprintf('    %-4s %8s %14s %12s %12s %12s %10s\n', 'scen', 'mean r', 'estimator', 'residual', 'lin. sum', 'measured', 'sw [mrad]');
for sc = {'A', 'B', 'C'}
    Rs = run_scenario(sc{1});  Lg = Rs.log;
    ideal = Lg.th + p.t_p*Lg.thd;                  % truth: available from t = 0
    ke = find(isfinite(Lg.th_pred), 1);            % first valid estimate
    e  = zeros(size(ideal));  e(ke:end) = Lg.th_pred(ke:end) - ideal(ke:end);
    y_est  = filter(nF, dF, e);
    y_det  = fz(ideal) - Lg.th;
    y_meas = Lg.th_turret - Lg.th;
    keep = Lg.t >= 2.5;  r_k = Lg.r(keep);
    m_ = @(x) sqrt(mean((r_k.*x(keep)).^2));
    s_w = std(diff(e(keep)))/sqrt(2);              % white-equivalent theta_pred noise
    fprintf('    %-4s %6.0f m %12.2f m %10.2f m %10.2f m %10.2f m %10.3f\n', sc{1}, mean(r_k), ...
        m_(y_est), m_(y_det), m_(y_est + y_det), m_(y_meas), 1e3*s_w);
end
fprintf('    sw = white-equivalent theta_pred noise; D24 spec <= 1.0 mrad.\n\n');

%% ---- A2: prediction error transmission ----
w  = logspace(-2, log10(pi/T), 2000);
[mag, ph] = bode(FT, w); mag = squeeze(mag); ph = squeeze(ph);
w3 = w(find(mag < 1/sqrt(2), 1));
lag = -ph(1)*pi/180 / w(1);
fprintf('A2  F(z)T(z): -3 dB at %.2f rad/s; |FT| = %.3f, %.3f, %.3f at 0.5, 1, 5 rad/s\n', ...
    w3, interp1(w, mag, 0.5), interp1(w, mag, 1), interp1(w, mag, 5));
fprintf('    Low-frequency lag = %.3f s (design target t_p = %.2f s)\n\n', lag, p.t_p);

fig = figure('Name', 'A2: F*T'); light_theme(fig);
semilogx(w, 20*log10(mag)); grid on; hold on
semilogx(w, 20*log10(abs(1./(1 + 1j*w*p.t_p))), '--');
xlabel('\omega [rad/s]'); ylabel('|F T| [dB]');
legend('F(z)T(z), 50 ms', '1/(1 + t_p s)', 'Location', 'southwest');
title('Prediction error \rightarrow turret angle');
exportgraphics(fig, fullfile(figdir, 'sens_FT.png'), 'Resolution', 200);

%% ---- A3: blackout-recovery step ----
% Worst case: constant-velocity coast through t_bo = 0.5 s with |a| = 20 m/s^2
% at the minimum range 800 m. On recovery the estimate corrects by
% d_theta = a t_bo^2 / (2 r) and d_thetadot = a t_bo / r, so theta_pred steps by
% d_theta + t_p d_thetadot.
d_th  = 0.5*p.a_max*p.t_bo_max^2 / p.r_min;
d_thd = p.a_max*p.t_bo_max / p.r_min;
stp   = d_th + p.t_p*d_thd;
fprintf('A3  Worst-case reference step on recovery: %.2f mrad (%.2f position + %.2f = t_p x rate)\n', ...
    1e3*stp, 1e3*d_th, 1e3*p.t_p*d_thd);
th0 = p.theta0_test;
ref = @(t) th0 + stp*(t >= 0.5);
[t3, x3, V3] = sim_grid(ref, [th0; 0; 0], 3, T, Dpf, th0);
k = find(abs(x3(:,1) - (th0 + stp)) > 1e-3 & t3 > 0.5, 1, 'last') + 1;
fprintf('    Turret within 1 mrad of the corrected reference %.2f s after recovery; peak |V| %.1f V, peak |i| %.2f A\n\n', ...
    t3(k) - 0.5, max(abs(V3)), max(abs(x3(:,3))));

fig = figure('Name', 'A3: recovery step'); light_theme(fig);
subplot(2,1,1); plot(t3, 1e3*(ref(t3) - th0), '--', t3, 1e3*(x3(:,1) - th0)); grid on
ylabel('[mrad]'); legend('\theta_{pred} step', 'turret'); title('Blackout recovery (worst case, r = 800 m)');
subplot(2,1,2); plot(t3, V3); grid on; ylabel('V_m [V]'); xlabel('t [s]');
exportgraphics(fig, fullfile(figdir, 'sens_recovery.png'), 'Resolution', 200);

%% ---- A4: acquisition vs slew size (voltage-saturated) ----
fprintf('A4  Linear region: |error| < V_max/K_P = %.3f rad (%.1f deg)\n', p.V_max/p.K_P, p.V_max/p.K_P/d2r);
slews = [10 30 47.5 95];
fig = figure('Name', 'A4: acquisition'); light_theme(fig); hold on; grid on
for sdeg = slews
    th_s = -10*d2r; th_e = th_s + sdeg*d2r;
    [t4, x4, V4] = sim_grid(@(t) th_e, [th_s; 0; 0], 5, T, [], th_e);  % [] = no prefilter
    k = find(abs(x4(:,1) - th_e) > 1e-3, 1, 'last') + 1;
    fprintf('    %5.1f deg slew: within 1 mrad at %.2f s, peak rate %.2f rad/s, peak |i| %.1f A\n', ...
        sdeg, t4(k), max(abs(x4(:,2))), max(abs(x4(:,3))));
    plot(t4, (x4(:,1) - th_s)/d2r);
end
xlabel('t [s]'); ylabel('\theta - \theta(0) [deg]'); title('Acquisition slews at \pm600 V');
legend(compose('%g deg', slews), 'Location', 'southeast');
exportgraphics(fig, fullfile(figdir, 'sens_acquisition.png'), 'Resolution', 200);
fprintf('\n');

%% ---- B: tracking voltage and current (analytic bounds) ----
% LOS rate <= v/r_min; LOS accel <= a/r_min + (v/r_min)^2 (since |r_dot| r theta_dot <= v^2/2)
thd  = p.v_max / p.r_min;
thdd = p.a_max / p.r_min + (p.v_max/p.r_min)^2;
V_tr = thd / p.Kvel;
i_tr = (p.J_eq*thdd + p.b*thd) / p.KtN;
fprintf('B   Tracking bounds: LOS rate <= %.3f rad/s -> V <= %.0f V; LOS accel <= %.3f rad/s^2 -> i <= %.2f A\n', ...
    thd, V_tr, thdd, i_tr);
fprintf('    Current is dominated by acquisition slews (A4), not by tracking.\n');
fprintf('\nFigures saved to %s\n', figdir);

%% ---------------- helpers ----------------
function g = h2_gain(sys)
% rms output per unit-variance white input sequence: sqrt(sum h_k^2)
    [n, d] = tfdata(sys, 'v');
    h = filter(n, d, [1 zeros(1, 20000)]);
    g = sqrt(sum(h.^2));
end

function [t, X, Vlog] = sim_grid(ref, x0, Tend, T, Dpf, xf0)
% Nonlinear plant (turret_plant, RK4 at 0.1 ms) with prefilter + feedback_step
% updated every T (D20). Dpf = [] bypasses the prefilter; otherwise
% xf_k = alpha xf_{k-1} + beta r_k + beta1 r_{k-1} (beta1 = 0 for the all-pole form).
    feedback_step('reset');
    h = 1e-4; n = round(Tend/h); every = round(T/h);
    t = (0:n)'*h; X = zeros(n+1, 3); Vlog = zeros(n+1, 1); X(1,:) = x0';
    xf = xf0; V = 0; rprev = xf0;
    for k = 1:n
        x = X(k,:)';
        if mod(k-1, every) == 0
            if isempty(Dpf)
                xf = ref(t(k));
            else
                rk = ref(t(k));
                xf = Dpf.alpha*xf + Dpf.beta*rk + Dpf.beta1*rprev;
                rprev = rk;
            end
            V  = feedback_step(xf, x(1), x(2), T);
        end
        Vlog(k) = V;
        k1 = f(x, V); k2 = f(x + h/2*k1, V); k3 = f(x + h/2*k2, V); k4 = f(x + h*k3, V);
        X(k+1,:) = (x + h/6*(k1 + 2*k2 + 2*k3 + k4))';
    end
    Vlog(end) = V;
end

function dx = f(x, V)
    [a, b, c] = turret_plant(0, V, x(1), x(2), x(3));
    dx = [a; b; c];
end

function light_theme(fig)
% Paper figures must print on white, even when MATLAB runs in dark mode (R2025a+).
    if isprop(fig, 'Theme'), fig.Theme = 'light'; end
end

