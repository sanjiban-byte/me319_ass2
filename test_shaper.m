% TEST_SHAPER  Unit test of the prefilter design (including the D38 lead-lag form).
%   (i)  n = 1 all-pole reproduces D23 exactly (alpha = tau_f/(tau_f + dt));
%        n = 1 lead-lag (D38): coefficients and equivalent pole / zero;
%   (ii) the chain F(e^{sT}) T(e^{sT}) P(s) equals 1 up to the matched order
%        (all-pole: s^n; lead-lag: s^2), with T the sampled loop at 50 ms;
%   (iii) time domain, n = 2 all-pole: constant-acceleration theta_hat is tracked
%        with zero steady error;
%   (iv) time domain, n = 1: constant-acceleration theta_hat. All-pole leaves the
%        D33 residual (about -0.046*theta_ddot); the lead-lag form removes it.
p = params(); dt = p.dt_ctrl; tp = p.t_p;
D1 = prefilter_design(1, dt, false);
fprintf('(i)  n=1 all-pole: alpha = %.6f, D23 tau_f/(tau_f+dt) = %.6f\n', D1.alpha, p.tau_f/(p.tau_f + dt));
DZ = prefilter_design(1, dt, true);
z0 = -DZ.beta1/DZ.beta;
fprintf(['     n=1 lead-lag: alpha = %.6f, beta = %.6f, beta1 = %+.6f (DC gain %.6f); ' ...
         'pole time constant %.4f s, zero time constant %.4f s\n'], DZ.alpha, DZ.beta, DZ.beta1, ...
         (DZ.beta + DZ.beta1)/(1 - DZ.alpha), -dt/log(DZ.alpha), -dt/log(z0));
designs = {{1, false}, {1, true}, {2, false}, {3, false}};
for q0 = 1:numel(designs)
    n = designs{q0}{1}; D = prefilter_design(n, dt, designs{q0}{2});
    M = 64; rho = 1; ph = 2*pi*(0:M-1)/M; s = rho*exp(1i*ph); g = zeros(1, M);
    for q = 1:M
        z = exp(s(q)*dt);
        F = sum(D.num.*z.^(-(0:numel(D.num)-1))) / (1 - sum(D.alpha.*z.^(-(1:numel(D.alpha)))));
        P = sum((tp*s(q)).^(0:n)./factorial(0:n));
        g(q) = F*D.Tz(z)*P - 1;
    end
    co = zeros(1, D.m+2);
    for m = 0:D.m+1, co(m+1) = real(mean(g.*exp(-1i*m*ph)))/rho^m; end
    if D.zero, nm = 'lead-lag'; else, nm = 'all-pole'; end
    fprintf('(ii) n=%d %s: coeffs of F*T*P-1, s^0..s^%d = [%s] (first %d must be ~0)  |pole| %.3f\n', ...
            n, nm, D.m+1, sprintf('%+.1e ', co), D.m+1, max(abs(D.poles)));
end

% sampled loop (ZOH plant, K_P, matched notch) for the time-domain checks
nx = 3; Mx = expm([p.A, p.B; zeros(1, nx+1)]*dt); Ad = Mx(1:nx,1:nx); Bd = Mx(1:nx,end);
sz = roots([1, 2*p.zeta_res*p.wn_res, p.wn_res^2]); sp = roots([1, 2*p.zeta_d*p.wn_res, p.wn_res^2]);
b = real(poly(exp(sz*dt))); an = real(poly(exp(sp*dt))); b = b*sum(an)/sum(b);

% (iii) n = 2 all-pole
e = track_(p, dt, tp, 2, Ad, Bd, b, an);
fprintf('(iii) n=2, constant-acceleration theta_hat: turret - theta_hat after 5 s = %.2e rad (max over 5-10 s %.2e)\n', ...
        e(end), max(abs(e(101:end))));
% (iv) n = 1, all-pole vs lead-lag
a = 0.004;
for zf = [false true]
    params('override', struct('prefilter_zero', zf));
    e = track_(p, dt, tp, 1, Ad, Bd, b, an);
    if zf, nm = 'lead-lag'; else, nm = 'all-pole'; end
    fprintf('(iv) n=1 %s, theta_ddot = %.3f rad/s^2: steady turret - theta_hat = %+.2e rad (%+.4f x theta_ddot)\n', ...
            nm, a, mean(e(101:end)), mean(e(101:end))/a);
end
params('override');

function e = track_(p, dt, tp, n, Ad, Bd, b, an)
% Polynomial theta_hat through predictor -> shaper -> linear sampled loop.
th0 = 0.2; w = 0.05; a = 0.004;                  % stays inside the stops
x = [th0; w; p.b*w/p.KtN]; wn = [0; 0]; K = 200; e = zeros(1, K);
for k = 0:K-1
    t = k*dt; th = th0 + w*t + 0.5*a*t^2; thd = w + a*t;
    pred = th + tp*thd;  if n >= 2, pred = pred + 0.5*tp^2*a; end
    est = struct('theta_hat', th, 'theta_dot_hat', thd, 'theta_ddot_hat', a, 'n', n, 'theta_pred', pred);
    r = reference_shaper(est, dt, k == 0);
    u = p.K_P*(r - x(1)); V = b(1)*u + wn(1);
    wn = [b(2)*u - an(2)*V + wn(2); b(3)*u - an(3)*V];
    e(k+1) = x(1) - th;
    x = Ad*x + Bd*V;
end
end
