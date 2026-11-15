%CHECK_TURRET_PLANT  Local verification of params.m, turret_plant.m, feedback_step.m.
%   Verification script (kept in tools/; reproduces the plant/loop checks reported in the paper).
%   Put this file next to the repo folder (or add the repo to the path) and run it.
%   Needs only base MATLAB (no toolboxes). Prints PASS/FAIL lines and makes plots.

clear params feedback_step   % reset persistent caches
clc; close all;

repo = fileparts(fileparts(mfilename('fullpath')));   % repository root (this file is in tools/)
if exist(repo, 'dir'), addpath(repo); end

p = params();
d2r = pi/180;
fprintf('=== Derived constants ===\n');
fprintf('J_eq = %.1f kg m^2 (expect 3200)\n', p.J_eq);
fprintf('Gp_den = [%g %g %g %g] (expect [320 3350 1.0015e6 0])\n', p.Gp_den);
fprintf('Kvel = %.6e rad/s/V (expect 9.985e-04)\n', p.Kvel);
fprintf('K_P = %.1f V/rad for Kv_fb = %g (expect ~8012), tau_f = %.3f s (expect 0.375)\n', p.K_P, p.Kv_fb, p.tau_f);
ev = eig(p.A);
fprintf('Plant eigenvalues: '); fprintf('%.3f%+.3fj  ', [real(ev) imag(ev)]'); fprintf('\n');
wn = abs(ev(imag(ev) > 0)); zeta = -real(ev(imag(ev) > 0))/wn;
report(abs(wn - 55.94) < 0.05 && abs(zeta - 0.0936) < 0.001, ...
    sprintf('Resonance wn = %.2f rad/s, zeta = %.4f (expect 55.94, 0.0936)', wn, zeta));

%% ---- Test 1: open-loop 10 V step, nonlinear plant vs exact linear solution ----
V0 = 10; T = 2.0; x0 = [p.theta0_test; 0; 0];   % 2 s so the 56 rad/s ripple has decayed
f = @(t, x) plant_vec(t, x, V0);
opts = odeset('RelTol', 1e-10, 'AbsTol', 1e-12);
[tt, xx] = ode15s(f, [0 T], x0, opts);
% exact linear response with constant input: x(t) = e^{At}x0 + int e^{A s} ds B V0
M = [p.A p.B*V0; zeros(1, 4)];
xl = zeros(numel(tt), 3);
for k = 1:numel(tt)
    E = expm(M*tt(k)); xl(k, :) = (E(1:3, 1:3)*x0 + E(1:3, 4))';
end
err = max(abs(xx(:, 1) - xl(:, 1)));
report(err < 1e-8, sprintf('Test 1: nonlinear vs linear theta max error = %.2e rad', err));
report(abs(xx(end, 2) - V0*p.Kvel) < 1e-6, ...
    sprintf('Test 1: final rate = %.6f rad/s (expect %.6f = V*Kvel)', xx(end, 2), V0*p.Kvel));
figure('Name', 'Test 1: open-loop 10 V step');
subplot(2,1,1); plot(tt, xx(:,2), tt, xl(:,2), '--'); grid on
ylabel('\theta dot [rad/s]'); legend('turret\_plant', 'linear'); title('10 V step: rate (56 rad/s ripple)');
subplot(2,1,2); plot(tt, xx(:,3)); grid on; ylabel('i [A]'); xlabel('t [s]');

%% ---- Test 2: hard stop, +600 V from 80 deg ----
x0 = [80*d2r; 0; 0];
[t2, x2] = sim_cl(@(t, th, thd) 600, x0, 1.5, 1e-3);
report(max(x2(:,1)) <= p.theta_max + 1e-3, ...
    sprintf('Test 2: max theta = %.4f deg (limit 85)', max(x2(:,1))/d2r));
report(abs(x2(end, 2)) < 1e-6, sprintf('Test 2: final rate at stop = %.2e rad/s', x2(end, 2)));
fprintf('        stall current at stop = %.1f A (V/R = 600 A expected)\n', x2(end, 3));
figure('Name', 'Test 2: hard stop');
subplot(3,1,1); plot(t2, x2(:,1)/d2r); grid on; ylabel('\theta [deg]'); title('+600 V into the 85 deg stop');
subplot(3,1,2); plot(t2, x2(:,2)); grid on; ylabel('\theta dot [rad/s]');
subplot(3,1,3); plot(t2, x2(:,3)); grid on; ylabel('i [A]'); xlabel('t [s]');

%% ---- Tests 3-5: closed loop with the notch design (D18), controller at 1 ms and 50 ms ----
for dtc = [1e-3 0.05]
    tag = sprintf('dt_ctrl = %g ms', dtc*1e3);
    th0 = p.theta0_test;

    % Test 3: small 1 deg step (linear region)
    feedback_step('reset');
    ref = @(t) th0 + 1*d2r;
    [t3, x3, V3] = sim_cl(@(t, th, thd) feedback_step(ref(t), th, thd, dtc), [th0;0;0], 2, dtc);
    y = (x3(:,1) - th0)/(1*d2r);
    t63 = t3(find(y >= 0.632, 1));
    os = max(y) - 1;
    fprintf('[%s] Test 3: 1 deg step: t63 = %.3f s (expect ~0.124), overshoot = %.1f %% (expect ~0 at 1 ms, ~6 at 50 ms), peak V = %.0f\n', ...
        tag, t63, 100*os, max(abs(V3)));

    % Test 4: large step 37.5 -> 80 deg (voltage-saturated slew)
    feedback_step('reset');
    ref = @(t) 80*d2r;
    [t4, x4, V4] = sim_cl(@(t, th, thd) feedback_step(ref(t), th, thd, dtc), [th0;0;0], 4, dtc);
    e4 = abs(x4(:,1) - 80*d2r);
    k_in = find(e4 > 1e-3, 1, 'last') + 1;
    fprintf('[%s] Test 4 (expect ~1.5 s, ~0.63 rad/s, 35-50 A): 42.5 deg slew: within 1 mrad at t = %.3f s, peak rate = %.3f rad/s, peak |i| = %.1f A, max |V| = %.0f V\n', ...
        tag, t4(min(k_in, end)), max(x4(:,2)), max(abs(x4(:,3))), max(abs(V4)));

    % Test 5: ramp 0.1 rad/s, steady lag should be 0.1/Kv_fb
    feedback_step('reset');
    w = 0.1; ref = @(t) th0 + w*t;
    [t5, x5] = sim_cl(@(t, th, thd) feedback_step(ref(t), th, thd, dtc), [th0;0;0], 3, dtc);
    lag = ref(t5(end)) - x5(end,1);
    fprintf('[%s] Test 5: ramp lag = %.4f rad (expect %.4f = w/Kv_fb)\n', tag, lag, w/p.Kv_fb);

    figure('Name', ['Closed loop, ' tag]);
    subplot(3,1,1); plot(t3, x3(:,1)/d2r); grid on; ylabel('\theta [deg]'); title(['1 deg step, ' tag]);
    subplot(3,1,2); plot(t4, x4(:,1)/d2r); grid on; ylabel('\theta [deg]'); title('37.5 \rightarrow 80 deg');
    subplot(3,1,3); plot(t4, V4); grid on; ylabel('V_m [V]'); xlabel('t [s]');
end

%% ---- Test 6: repeated / earlier call times must not update the controller ----
feedback_step('reset');
v1 = feedback_step(p.theta0_test + 0.01, p.theta0_test, 0, 1e-3);
v2 = feedback_step(p.theta0_test + 0.05, p.theta0_test, 0, 0);     % dt = 0 -> hold
v3 = feedback_step(p.theta0_test + 0.05, p.theta0_test, 0, -1e-3); % dt < 0 -> hold
report(v2 == v1 && v3 == v1, sprintf('Test 6: dt <= 0 holds output (%.2f, %.2f, %.2f V)', v1, v2, v3));

%% ---------------- helpers ----------------
function dx = plant_vec(t, x, V)
    [a, b, c] = turret_plant(t, V, x(1), x(2), x(3));
    dx = [a; b; c];
end

function [t, X, Vlog] = sim_cl(ctrl, x0, T, dtc)
% Fixed-step RK4 (h = 0.1 ms) plant integration; controller sampled every dtc (ZOH).
    h = 1e-4; n = round(T/h); every = max(1, round(dtc/h));
    t = (0:n)'*h; X = zeros(n+1, 3); Vlog = zeros(n+1, 1);
    X(1,:) = x0'; V = 0;
    for k = 1:n
        x = X(k,:)';
        if mod(k-1, every) == 0, V = ctrl(t(k), x(1), x(2)); end
        Vlog(k) = V;
        k1 = pv(t(k), x, V); k2 = pv(t(k)+h/2, x+h/2*k1, V);
        k3 = pv(t(k)+h/2, x+h/2*k2, V); k4 = pv(t(k)+h, x+h*k3, V);
        X(k+1,:) = (x + h/6*(k1 + 2*k2 + 2*k3 + k4))';
    end
    Vlog(end) = V;
end

function dx = pv(t, x, V)
    [a, b, c] = turret_plant(t, V, x(1), x(2), x(3));
    dx = [a; b; c];
end

function report(ok, msg)
    if ok, fprintf('PASS  %s\n', msg); else, fprintf('FAIL  %s\n', msg); end
end
