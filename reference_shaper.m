function [theta_ref, info] = reference_shaper(est, dt, reset)
%REFERENCE_SHAPER  Prefilter F(z) and elevation clamp (owner: Harsh, T2).
%   [theta_ref, info] = reference_shaper(est, dt, reset)
%   est   estimator struct (theta_pred, theta_hat, theta_dot_hat,
%         theta_ddot_hat, n); dt time since the previous update [s]
%         (0.05 s on the D20 grid); reset true on the first update of a run.
%
%   Design rule (D2, D4): predictor lead P(s) of order n, prefilter F and the
%   sampled feedback loop T together give F*T*P = 1 + O(s^{n+1}), so the
%   turret points at the current estimate theta_hat(t). The coefficients come
%   from prefilter_design(n, dt), which uses the actual sampled loop
%   (K_P + matched notch, ZOH plant) and realises F as
%       y_k = sum_{j=1..n} alpha_j y_{k-j} + beta u_k .
%   For n = 1 this is exactly D23 (alpha = tau_f/(tau_f+dt), tau_f = 0.375 s).
%   With p.prefilter_zero (D38, n = 1) F gets one zero, y_k = alpha y_{k-1} +
%   beta u_k + beta1 u_{k-1}, matched up to s^2, which removes the D33 residual.
%
%   Initial condition (D4, generalised to order n): the past outputs are set to
%   the steady response of F to the current polynomial input,
%   y(t+tau) = sum_m f_m u^(m)(t+tau), f = Taylor coefficients of 1/Q(s),
%   u = theta_pred, u^(m) = sum_l t_p^l/l! theta_hat^(m+l)  (m >= 1).
%   For n = 1 this is x0 = theta_hat + theta_dot_hat/Kv_fb. The IC is
%   re-applied for t_warm seconds while the estimate settles.
%   Clamp: input and output limited to the elevation range (D6).
persistent Y D t_acc U
p = params();
if reset || isempty(Y), Y = []; D = []; t_acc = 0; U = []; end

n = p.n_pred;
if isfield(est, 'n') && ~isempty(est.n) && est.n >= 1, n = min(round(est.n), 3); end
if isempty(D) || D.n ~= n || abs(D.dt - dt) > 1e-6
    if dt <= 0, dt = p.dt_ctrl; end
    D = prefilter_design(n, dt);
    if ~isempty(Y) && numel(Y) ~= n, Y = []; end
end

u = min(max(est.theta_pred, p.theta_min), p.theta_max);

if isempty(Y) || t_acc < p.t_warm - 1e-9
    dv = zeros(1, n+1);                         % theta_hat derivatives
    dv(1) = est.theta_hat;
    if n >= 1, dv(2) = est.theta_dot_hat;  end
    if n >= 2, dv(3) = est.theta_ddot_hat; end
    ud = zeros(1, n+1);  ud(1) = u;             % u and its derivatives at t
    for m = 1:n
        for l = 0:n-m, ud(m+1) = ud(m+1) + p.t_p^l/factorial(l)*dv(m+l+1); end
    end
    Y = zeros(n, 1);                            % Y(j) = y_{k-j} = y_ss(t - j dt)
    for j = 1:n
        tau = -j*D.dt;
        for m = 0:n
            um = 0;                             % u^(m)(t + tau), Taylor to order n
            for l = 0:n-m, um = um + ud(m+l+1)*tau^l/factorial(l); end
            Y(j) = Y(j) + D.f(m+1)*um;
        end
    end
    U = 0;                                      % u_{k-1} = u(t - dt), Taylor (lead-lag form, D38)
    for l = 0:n, U = U + ud(l+1)*(-D.dt)^l/factorial(l); end
end
t_acc = t_acc + dt;

y = D.alpha*Y + D.beta*u + D.beta1*U;          % beta1 = 0 for the all-pole form
Y = [y; Y(1:end-1)];
U = u;

theta_ref = min(max(y, p.theta_min), p.theta_max);          % D6 clamp
info = struct('theta_ref_raw', y, 'clamped', theta_ref ~= y, ...
              'warmup', t_acc <= p.t_warm + 1e-9, 'D', D);
end
