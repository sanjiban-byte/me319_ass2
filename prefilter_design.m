function D = prefilter_design(n, dt, use_zero)
%PREFILTER_DESIGN  Discrete prefilter F(z) that cancels the predictor lead.
%   D = prefilter_design(n, dt)             zero on/off from p.prefilter_zero
%   D = prefilter_design(n, dt, use_zero)   (owner: Harsh, T2; base MATLAB only)
%
%   Rule (D2/D4): the whole chain predictor -> prefilter -> sampled loop must
%   satisfy  F(e^{sT}) * T(e^{sT}) * P(s) = 1 + O(s^{m+1}),
%   P(s) = sum_{k=0..n} (t_p s)^k/k!, so that the turret points at the current
%   estimate. Two realisations:
%
%   All-pole (D23/D26), matched to m = n:
%     y_k = sum_{j=1..n} alpha_j y_{k-j} + beta u_k.
%     For n = 1 this is exactly D23: alpha = tau_f/(tau_f + dt). It leaves the
%     D33 residual F*T*P - 1 ~ -0.046 s^2 (pointing error ~ 0.046 r theta_ddot).
%
%   Lead-lag with one zero (D38, n = 1 only), matched to m = 2:
%     y_k = alpha y_{k-1} + beta u_k + beta1 u_{k-1}.
%     Three coefficients match F*T*P = 1 up to s^2, which removes the D33
%     residual. For an ideal loop T = 1/(1 + s/Kv) this is the discrete form of
%     F = (1 + s/Kv)/(1 + t_p s).
%
%   Steps: 1. a(m) = Taylor coefficients of 1/T(e^{sT}) of the linear sampled
%   loop (ZOH plant at dt, K_P, matched notch), by a Cauchy contour integral
%   (exact up to round-off); a1 = 1/Kv_fb. 2. Target coefficients of F.
%   3. Discrete realisation by moment matching.
%
%   D fields: n, dt, zero, m (matched order), a, c (Q coeffs, all-pole),
%   alpha, beta, beta1 (0 if no zero), num/den (F(z) in powers of z^-1),
%   poles, f (Taylor coeffs of F(s), for initial conditions), Tz.
p = params();
tp = p.t_p;
if nargin < 3
    use_zero = isfield(p, 'prefilter_zero') && p.prefilter_zero && n == 1;
end
if use_zero && n ~= 1
    error('prefilter_design: the lead-lag (zero) form is implemented for n = 1 only');
end
mo = n + double(use_zero);                   % order matched

% ---- 1. sampled closed loop, linear (no saturation) --------------------
nx = 3;  M = expm([p.A, p.B; zeros(1, nx+1)]*dt);
Ad = M(1:nx,1:nx);  Bd = M(1:nx,nx+1);  C = p.C;
sz = roots([1, 2*p.zeta_res*p.wn_res, p.wn_res^2]);
sp = roots([1, 2*p.zeta_d  *p.wn_res, p.wn_res^2]);
b = real(poly(exp(sz*dt)));  an = real(poly(exp(sp*dt)));  b = b*sum(an)/sum(b);
K = p.K_P;
% states X = [x; w1; w2];  u = K (r - C x);  V = b1 u + w1
%  w1+ = b2 u - a2 V + w2 ;  w2+ = b3 u - a3 V ;  x+ = Ad x + Bd V
Vx = [-b(1)*K*C, 1, 0];  Vr = b(1)*K;              % V = Vx X + Vr r
ux = [-K*C, 0, 0];       ur = K;
Acl = [Ad*eye(nx), zeros(nx,2); zeros(2, nx+2)] + [Bd; -an(2); -an(3)]*Vx;
Acl(nx+1,:) = Acl(nx+1,:) + b(2)*ux + [zeros(1,nx), 0, 1];
Acl(nx+2,:) = Acl(nx+2,:) + b(3)*ux;
Bcl = [Bd*Vr; b(2)*ur - an(2)*Vr; b(3)*ur - an(3)*Vr];
Ccl = [C, 0, 0];
if max(abs(eig(Acl))) >= 1, error('prefilter_design: sampled loop unstable at dt = %g', dt); end
Tz = @(z) Ccl*((z*eye(nx+2) - Acl)\Bcl);

% Cauchy integral on a small circle in the s-plane
Mpts = 64; rho = 1.0; phi = 2*pi*(0:Mpts-1)/Mpts; s = rho*exp(1i*phi);
g = zeros(size(s));
for q = 1:Mpts, g(q) = 1/Tz(exp(s(q)*dt)); end
a = zeros(1, mo+1);
for m = 0:mo, a(m+1) = real(mean(g.*exp(-1i*m*phi)))/rho^m; end

if use_zero
    % ---- 2z. target F(s) = 1/(T(s) P(s)) up to s^2 ------------------------
    Pfull = zeros(1, mo+1);  Pfull(1:n+1) = tp.^(0:n)./factorial(0:n);
    g = conv(a, series_inv_(Pfull));  g = g(1:mo+1);    % Taylor coeffs of F
    % ---- 3z. y_k = alpha y_{k-1} + beta u_k + beta1 u_{k-1} --------------
    % (1 - alpha e^{-s dt}) G(s) = beta + beta1 e^{-s dt}, coefficients of s^0..s^2
    E = (-dt).^(0:mo)./factorial(0:mo);
    Aeq = zeros(mo+1, 3);  rhs = g(:);
    for m = 0:mo
        Aeq(m+1, 1) = sum(E(1:m+1).*g(m+1:-1:1));      % alpha
        Aeq(m+1, 2) = (m == 0);                          % beta
        Aeq(m+1, 3) = E(m+1);                            % beta1
    end
    sol = Aeq\rhs;  alpha = sol(1);  beta = sol(2);  beta1 = sol(3);
    poles = alpha;
    if abs(alpha) >= 1
        error('prefilter_design: lead-lag prefilter unstable (dt=%g)', dt);
    end
    D = struct('n', n, 'dt', dt, 'zero', true, 'm', mo, 'a', a, 'c', [], ...
               'alpha', alpha, 'beta', beta, 'beta1', beta1, ...
               'num', [beta, beta1], 'den', [1, -alpha], 'poles', poles, ...
               'f', g, 'loop_poles', eig(Acl));
else
% ---- 2. continuous prefilter Q(s) = P(s) * T  (series, truncated) -------
P = tp.^(0:n)./factorial(0:n);
Tser = series_inv_(a);                     % Taylor coefficients of T
c = conv(P, Tser);  c = c(1:n+1);

% ---- 3. discrete realisation by moment matching ------------------------
% rows m = 0..n:  sum_j alpha_j (-j dt)^m/m! + beta c_m = delta_m0
Aeq = zeros(n+1);  rhs = zeros(n+1,1);  rhs(1) = 1;
for m = 0:n
    for j = 1:n, Aeq(m+1, j) = (-j*dt)^m/factorial(m); end
    Aeq(m+1, n+1) = c(m+1);
end
sol = Aeq\rhs;  alpha = sol(1:n).';  beta = sol(n+1);
poles = roots([1, -alpha]);
if any(abs(poles) >= 1)
    error('prefilter_design: discrete prefilter unstable (n=%d, dt=%g)', n, dt);
end
D = struct('n', n, 'dt', dt, 'zero', false, 'm', mo, 'a', a, 'c', c, ...
           'alpha', alpha, 'beta', beta, 'beta1', 0, ...
           'num', beta, 'den', [1, -alpha], 'poles', poles, ...
           'f', series_inv_(c), 'loop_poles', eig(Acl));
end
D.Tz = Tz;                                 % sampled loop T(z), for tests
end

function y = series_inv_(x)
% Taylor coefficients of 1/x(s), same length as x (x(1) ~= 0)
n = numel(x); y = zeros(1, n); y(1) = 1/x(1);
for k = 2:n, y(k) = -sum(x(2:k).*y(k-1:-1:1))/x(1); end
end
