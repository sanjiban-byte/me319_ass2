function R = run_scenario(scenario, opts)
%RUN_SCENARIO  Closed-loop simulation of one engagement (owner: Harsh, T2).
%   R = run_scenario(scenario, opts)
%   opts (all optional): dt_call, dt_sim, th0 [rad], verbose.
%   The plant (turret_plant.m, Jithu) is integrated with fixed-step RK4 at
%   dt_sim (1 ms: below the 5 ms hard-stop time constant and resolving the
%   56 rad/s mode). The controller is called every dt_call (default 0.05 s);
%   it updates internally on its own 0.05 s grid (D20), so results should not
%   depend on dt_call. Hard stops live inside turret_plant. Pulses follow the evaluator rule:
%   a 0->1 edge counts only if >= t_recharge has elapsed since the last pulse.
p = params();
if nargin < 2, opts = struct(); end
dtc = getf_(opts,'dt_call', p.dt_ctrl);  dts = getf_(opts,'dt_sim', p.dt_sim);
th0 = getf_(opts,'th0', p.theta0_test);
nsub = max(1, round(dtc/dts));  dts = dtc/nsub;
Nc = floor(p.t_end/dtc + 1e-9);
z = [th0; 0; 0];                                   % theta, thetadot, current
prevfire = 0; t_pulse_last = -Inf; pulses = struct('t',{},'miss',{},'hit',{});
L = struct();  fn = {'t','x','y','r','th','thd','r_meas','th_meas','valid','th_hat','r_hat', ...
    'thd_hat','th_pred','sig','coast','th_ref','th_turret','thd_turret','V','i','fire','err_m','err_rad','d_hat','phit','gate'};
for q = 1:numel(fn), L.(fn{q}) = nan(1, Nc+1); end
clear turret_controller                            % fresh persistent state
ipk = 0;
% plant-step log (dt_sim): V, i, theta, theta_dot. The 50 ms log misses current
% peaks inside a control period (e.g. the 35 A peak at saturation onset).
nf = Nc*nsub + 1;  F = struct('t', zeros(1,nf), 'V', zeros(1,nf), 'i', zeros(1,nf), ...
    'th', zeros(1,nf), 'thd', zeros(1,nf));  jf = 1;
F.th(1) = z(1);  F.thd(1) = z(2);  F.i(1) = z(3);
for k = 0:Nc
    t = k*dtc;
    [rm, tm, mv] = target_motion_generator(t, scenario);
    [V, fire] = turret_controller(t, rm, tm, mv, z(1), z(2));
    d = turret_debug('get');
    [x,y,r,th,vx,vy] = target_truth(t, scenario);
    % --- pulses ---------------------------------------------------------
    if fire == 1 && prevfire == 0 && (t - t_pulse_last) >= p.t_recharge - 1e-9
        t_pulse_last = t;
        ux = cos(z(1)); uy = sin(z(1)); s = x*ux + y*uy;
        if s >= 0, miss = abs(x*uy - y*ux); else, miss = hypot(x,y); end
        pulses(end+1) = struct('t', t, 'miss', miss, 'hit', miss <= p.R_eng); %#ok<AGROW>
    end
    prevfire = fire;
    % --- log ------------------------------------------------------------
    j = k+1; err = angdiff_(z(1), th);
    L.t(j)=t; L.x(j)=x; L.y(j)=y; L.r(j)=r; L.th(j)=th; L.thd(j)=(x*vy-y*vx)/r^2;
    L.r_meas(j)=rm; L.th_meas(j)=tm; L.valid(j)=mv;
    L.th_hat(j)=d.est.theta_hat; L.r_hat(j)=d.est.r_hat; L.thd_hat(j)=d.est.theta_dot_hat;
    L.th_pred(j)=d.est.theta_pred; L.sig(j)=d.est.sigma_cross; L.coast(j)=d.est.coasting;
    L.th_ref(j)=d.theta_ref; L.th_turret(j)=z(1); L.thd_turret(j)=z(2);
    L.d_hat(j)=d.fire.d_hat; L.phit(j)=d.fire.phit; L.gate(j)=d.fire.gate;
    F.V(jf)=V;
    L.V(j)=V; L.i(j)=z(3); L.fire(j)=fire; L.err_rad(j)=err; L.err_m(j)=r*sin(err);
    if k == Nc, break; end
    % --- plant: RK4 over one control period, V held ----------------------
    for m = 1:nsub
        z = rk4_(@(zz) plant_rhs_(t, V, zz), z, dts);
        ipk = max(ipk, abs(z(3)));
        jf = jf + 1;  F.t(jf) = t + m*dts;  F.V(jf) = V;
        F.th(jf) = z(1);  F.thd(jf) = z(2);  F.i(jf) = z(3);
    end
end
R = struct('log', L, 'fine', F, 'pulses', pulses, 'scenario', scenario, 'opts', ...
           struct('dt_call', dtc, 'dt_sim', dts, 'th0', th0));
R.metrics = metrics_(L, pulses, p);
R.metrics.peak_i = max(R.metrics.peak_i, ipk);
if getf_(opts,'verbose', false), print_metrics(R); end
end

% ---------------------------------------------------------------------------
function dz = plant_rhs_(t, V, z)
[a, b, c] = turret_plant(t, V, z(1), z(2), z(3));
dz = [a; b; c];
end
function z = rk4_(f, z, h)
k1 = f(z); k2 = f(z + h/2*k1); k3 = f(z + h/2*k2); k4 = f(z + h*k3);
z = z + h/6*(k1 + 2*k2 + 2*k3 + k4);
end
function v = getf_(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end
function d = angdiff_(a, b)
d = atan2(sin(a-b), cos(a-b));
end
function M = metrics_(L, pulses, p)
M.n_pulses = numel(pulses);
hit = [pulses.hit];
M.success = any(hit);
if M.success, tp_ = [pulses.t]; M.t_first_hit = tp_(find(hit,1)); else, M.t_first_hit = NaN; end
M.miss = [pulses.miss];
M.best_miss = min([M.miss, Inf]);
M.n_hits = sum(hit);
reach = L.th >= p.theta_min & L.th <= p.theta_max;     % score only while target is reachable
lock = find(abs(L.err_m) <= p.R_eng, 1);
if isempty(lock), M.t_lock = NaN; M.rms_err_m = NaN; M.max_err_m = NaN; M.rms_err_mrad = NaN;
    M.rms_ctrl_m = NaN; M.rms_est_m = NaN; M.mean_ctrl_m = NaN;
else
    M.t_lock = L.t(lock);
    ix = false(size(L.t)); ix(lock:end) = true; ix = ix & reach;
    e = L.err_m(ix);  M.rms_err_m = sqrt(mean(e.^2));  M.max_err_m = max(abs(e));
    M.rms_err_mrad = 1e3*sqrt(mean(L.err_rad(ix).^2));
    ec = L.r(ix).*sin(L.th_turret(ix) - L.th_hat(ix));   % control part: turret vs estimate
    ee = L.r(ix).*sin(L.th_hat(ix) - L.th(ix));          % estimation part: estimate vs truth
    M.rms_ctrl_m = sqrt(mean(ec.^2));  M.rms_est_m = sqrt(mean(ee.^2));
    M.mean_ctrl_m = mean(ec);
end
M.peak_V = max(abs(L.V));  M.peak_i = max(abs(L.i));
M.rms_V = sqrt(mean(L.V.^2)); M.rms_i = sqrt(mean(L.i.^2));
end
