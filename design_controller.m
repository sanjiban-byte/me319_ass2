%DESIGN_CONTROLLER  Analysis and figures for the turret feedback controller Gc(s).
%   Reproduces the controller-design results reported in the paper
%   (decision D18): plant Gp(s), compensator Gc(s) = K_P * N(s), stability
%   margins in continuous time and at sampled call rates, root locus, step and
%   ramp responses, and robustness to plant-parameter errors.
%   Requires the Control System Toolbox. Figures are saved to ./figures.

clear params                           % pick up the current params.m
p = params();
s = tf('s');
figdir = fullfile(fileparts(mfilename('fullpath')), 'figures');
if ~exist(figdir, 'dir'), mkdir(figdir); end

%% ---- Plant and compensator ----
Gp = tf(p.Gp_num, p.Gp_den);                       % theta/V
wn = p.wn_res; zr = p.zeta_res; zd = p.zeta_d;
N  = (s^2 + 2*zr*wn*s + wn^2) / (s^2 + 2*zd*wn*s + wn^2);   % notch, N(0) = 1
Gc = p.K_P * N;
L  = Gc * Gp;                                      % loop transfer function
T  = feedback(L, 1);                               % reference -> turret

fprintf('Plant: wn = %.2f rad/s, zeta = %.4f, Kvel = %.4e rad/s/V\n', wn, zr, p.Kvel);
fprintf('Design: Kv_fb = %g 1/s, K_P = %.1f V/rad, zeta_d = %.2f\n', p.Kv_fb, p.K_P, zd);
fprintf('Prefilter: tau_f = t_p - 1/Kv_fb = %.3f s\n\n', p.tau_f);

%% ---- Margins: P-only vs notch, continuous ----
fprintf('%-18s %8s %8s %10s %10s\n', 'Design', 'GM [dB]', 'PM [deg]', 'wc [rad/s]', 'max Re(p)');
for Kv = [5 8 10]
    Lp = (Kv/p.Kvel) * Gp;
    [gm, pm, ~, wc] = margin(Lp);
    fprintf('%-18s %8.1f %8.1f %10.2f %10.2f\n', sprintf('P, Kv=%g', Kv), 20*log10(gm), pm, wc, max(real(pole(feedback(Lp,1)))));
end
[gm, pm, ~, wc] = margin(L);
fprintf('%-18s %8.1f %8.1f %10.2f %10.2f\n\n', sprintf('Notch, Kv=%g', p.Kv_fb), 20*log10(gm), pm, wc, max(real(pole(T))));

%% ---- Sampled loop: ZOH plant, matched-notch controller ----
fprintf('Sampled loop (ZOH plant, pole-zero matched notch):\n');
fprintf('%-10s %8s %8s %12s\n', 'dt [s]', 'GM [dB]', 'PM [deg]', 'max |z|');
for dt = [1e-3 1e-2 5e-2]
    Ld = p.K_P * c2d(N, dt, 'matched') * c2d(Gp, dt, 'zoh');
    [gm, pm] = margin(Ld);
    fprintf('%-10g %8.1f %8.1f %12.4f\n', dt, 20*log10(gm), pm, max(abs(pole(feedback(Ld,1)))));
end
fprintf('(max |z| is the cancelled plant mode e^{-zeta wn dt}; it is not excited by V.)\n\n');

%% ---- Robustness to +/-20%% inertia and inductance errors ----
fprintf('Robustness (controller fixed, plant perturbed):\n');
fprintf('%-6s %-6s %8s %8s\n', 'J x', 'L x', 'GM [dB]', 'PM [deg]');
for f = [1 1; 0.8 1; 1.2 1; 1 0.8; 1 1.2; 0.8 1.2; 1.2 0.8]'
    J2 = p.J_eq*f(1); L2 = p.L*f(2);
    Gp2 = tf(p.KtN, [conv([J2 p.b], [L2 p.R]) + [0 0 p.KtN*p.KeN], 0]);
    [gm, pm] = margin(Gc*Gp2);
    fprintf('%-6.1f %-6.1f %8.1f %8.1f\n', f(1), f(2), 20*log10(gm), pm);
end

%% ---- Figures ----
Lp8 = (p.Kv_fb/p.Kvel) * Gp;

fig = figure('Name', 'Bode'); light_theme(fig);
bode(Gp, Lp8, L, {1e-1, 1e3}); grid on
legend('G_p(s)', 'P only, K_v = 8', 'K_P N(s) G_p(s)', 'Location', 'southwest');
title('Open-loop frequency response');
exportgraphics(fig, fullfile(figdir, 'bode_loop.png'), 'Resolution', 200);

fig = figure('Name', 'Root locus'); light_theme(fig);
subplot(1,2,1); rlocus(Gp); hold on
pc = pole(feedback(Lp8, 1)); plot(real(pc), imag(pc), 'ks', 'MarkerFaceColor', 'k');
title('P control only (squares: closed-loop poles at Kv = 8)', 'Interpreter', 'none');
ws = warning('off', 'all'); xlim([-80 20]); ylim([-80 80]); warning(ws);
subplot(1,2,2); rlocus(N*Gp); hold on
pc = pole(T); plot(real(pc), imag(pc), 'ks', 'MarkerFaceColor', 'k');
title('Notch controller KP*N(s) (squares: design closed-loop poles)', 'Interpreter', 'none');
ws = warning('off', 'all'); xlim([-80 20]); ylim([-80 80]); warning(ws);
exportgraphics(fig, fullfile(figdir, 'root_locus.png'), 'Resolution', 200);

fig = figure('Name', 'Linear responses'); light_theme(fig);
tt = 0:1e-3:1.5;
subplot(2,1,1); step(feedback(Lp8,1), T, tt); grid on
legend('P only, K_v = 8', 'K_P N(s)'); title('Unit step (linear, unsaturated)');
subplot(2,1,2);
r = 0.1*tt; y = lsim(T, r, tt);
plot(tt, r - y'); grid on; xlabel('t [s]'); ylabel('error [rad]');
title(sprintf('0.1 rad/s ramp: steady error = 0.1/K_v = %.4f rad', 0.1/p.Kv_fb));
exportgraphics(fig, fullfile(figdir, 'step_ramp.png'), 'Resolution', 200);

fprintf('\nFigures saved to %s\n', figdir);

function light_theme(fig)
% Paper figures must print on white, even when MATLAB runs in dark mode (R2025a+).
    if isprop(fig, 'Theme'), fig.Theme = 'light'; end
end
