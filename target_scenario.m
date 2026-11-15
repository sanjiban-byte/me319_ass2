function S = target_scenario(scenario)
%TARGET_SCENARIO  Build (and cache) the truth trajectory and blackout window.
%   S = target_scenario(scenario)
%   scenario: 'A' | 'B' | 'C' (or 1|2|3), or an integer >= 4 for a seeded
%   random bounded-maneuver case (used for generalisation tests). An id of
%   1000 + k is case k with a forced full 0.5 s blackout (blackout stress test).
%   Fields: t, x, y, vx, vy, ax, ay (1 ms grid, 0..T), t_boo, t_bo, id.
%   Range, speed and acceleration limits are checked on the whole trajectory
%   and an error is raised if any is violated.
persistent cache
if isempty(cache), cache = containers.Map('KeyType','char','ValueType','any'); end
p = params();
if isstruct(scenario)                       % designed scenario (scenario_library.m)
    key = sprintf('%s|%d', scenario.name, p.noise_seed);
    if isKey(cache, key), S = cache(key); return; end
    S = build_spec_(scenario, p);
    cache(key) = S;
    return
end
id = scenario_id_(scenario);
key = sprintf('%d|%d', id, p.noise_seed);   % noise depends on the seed too
if isKey(cache, key), S = cache(key); return; end

h = 1e-3;  T = 12;  t = (0:h:T).';
w = @(tau) smooth_ramp_(tau, 1.0);          % C2 onset over 1 s

if id <= 3                                   % standardized scenarios
    x0 = 1800; y0 = 400; vx0 = -95; vy0 = 10;
    switch id
        case 1, Ax = 0;    Ay = 0;    wx = 1;   wy = 1;   fx = 0; fy = 0;
        otherwise, Ax = 13; Ay = 13;  wx = 1.2; wy = 1.7; fx = 0; fy = pi/3;
    end
    t_man = 1.0;
    if id == 3, t_boo = 1.0; t_bo = 0.5; else, t_boo = Inf; t_bo = 0; end
else                                         % seeded random case
    base = id;  stress_bo = id > 1000;       % id 1000+k: case k with a forced
    if stress_bo, base = id - 1000; end      % full 0.5 s blackout (stress test)
    for attempt = 0:199
        g  = lcg_(base*1000 + attempt);
        rr = 1300 + 600*g(1);  an = (12 + 38*g(2))*pi/180;
        sp = 60 + 60*g(3);     hd = pi + (g(4)-0.5)*1.2;   % mostly inbound
        x0 = rr*cos(an); y0 = rr*sin(an);
        vx0 = sp*cos(hd); vy0 = sp*sin(hd);
        am = 19*(0.4 + 0.6*g(5));  ga = g(6)*pi/2;
        Ax = am*cos(ga); Ay = am*sin(ga);
        wx = 0.5 + 1.5*g(7); wy = 0.5 + 1.5*g(8);
        fx = 2*pi*g(9); fy = 2*pi*g(10);
        t_man = 1.0 + 1.5*g(11);
        t_boo = 1.0 + 7.5*g(12); t_bo = 0.5*g(13);
        if g(14) < 0.25, t_bo = 0; t_boo = Inf; end   % some cases: no blackout
        if stress_bo, t_bo = p.t_bo_max; t_boo = 1.0 + 7.5*g(12); end
        [ok, ~] = trajectory_ok_(x0,y0,vx0,vy0,Ax,Ay,wx,wy,fx,fy,t_man,t,h,w,p);
        if ok, break; end
    end
    if ~ok, error('target_scenario: no admissible random case for id %d', id); end
end

[x,y,vx,vy,ax,ay] = integrate_(x0,y0,vx0,vy0,Ax,Ay,wx,wy,fx,fy,t_man,t,h,w);
r = hypot(x,y); v = hypot(vx,vy); a = hypot(ax,ay);
m = t <= p.t_end;                            % limits apply on 0..t_end
if min(r(m)) < p.r_min-1e-9 || max(r(m)) > p.r_max+1e-9 || ...
   max(v(m)) > p.v_max+1e-9 || max(a(m)) > p.a_max+1e-9
    error('target_scenario: envelope violated (r %.0f-%.0f m, v %.1f, a %.2f)', ...
          min(r(m)), max(r(m)), max(v(m)), max(a(m)));
end
% Radar noise: independent N(0,1) draws per 20 Hz sample (white, as in the
% spec). Drawn once per scenario from a seeded Mersenne Twister; the global
% RNG state is saved and restored, so callers are unaffected. (Replaces the
% earlier per-sample Lehmer hash, whose consecutive draws were correlated:
% lag-1 autocorrelation 0.69 in theta.)
nk = floor(T/p.dt_radar) + 2;
rs_ = rng;
rng(p.noise_seed + 7919*id, 'twister');
nz = randn(nk, 2);
rng(rs_);
if t_bo > 0, bo = [t_boo, t_boo + t_bo]; else, bo = zeros(0, 2); end
S = struct('t',t,'x',x,'y',y,'vx',vx,'vy',vy,'ax',ax,'ay',ay, ...
           't_boo',t_boo,'t_bo',t_bo,'bo',bo,'id',id, 'noise', nz);
cache(key) = S;
end

% ---------------------------------------------------------------------------
function S = build_spec_(sp, p)
% Designed trajectory. sp fields: name, id (integer, seeds the noise), r0 [m],
% th0 [deg], v0 [m/s], psi0 [deg, heading from +x], and either
%   acc_t(t), acc_n(t): tangential / left-normal acceleration [m/s^2]
%                       (coordinated turns, jinks; speed changes only via acc_t)
%   acc_xy(t):          N x 2 Cartesian acceleration (weaves)
% plus bo (k x 2 blackout windows [start end], s). Integrated on a 1 ms grid
% (RK2). Envelope (range, speed, acceleration on 0..t_end) and turret reach
% (theta inside the elevation limits) are checked; a violation is an error.
h = 1e-3;  T = 12;  t = (0:h:T).';  n = numel(t);
P = zeros(n, 2);  V = zeros(n, 2);  A = zeros(n, 2);
P(1,:) = sp.r0*[cosd(sp.th0), sind(sp.th0)];
V(1,:) = sp.v0*[cosd(sp.psi0), sind(sp.psi0)];
if isfield(sp, 'acc_xy') && isa(sp.acc_xy, 'function_handle')
    A = sp.acc_xy(t);
    for k = 1:n-1
        V(k+1,:) = V(k,:) + h/2*(A(k,:) + A(k+1,:));
        P(k+1,:) = P(k,:) + h/2*(V(k,:) + V(k+1,:));
    end
else
    at = sp.acc_t(t);  an = sp.acc_n(t);
    acc = @(v, k) at(k)*v/norm(v) + an(k)*[-v(2), v(1)]/norm(v);
    for k = 1:n-1
        a1 = acc(V(k,:), k);
        vm = V(k,:) + h*a1;
        a2 = acc(vm, k+1);
        V(k+1,:) = V(k,:) + h/2*(a1 + a2);
        P(k+1,:) = P(k,:) + h/2*(V(k,:) + V(k+1,:));
        A(k,:) = a1;
    end
    A(n,:) = acc(V(n,:), n);
end
m = t <= p.t_end;
r = hypot(P(:,1), P(:,2));  th = atan2(P(:,2), P(:,1));
vv = hypot(V(:,1), V(:,2));  aa = hypot(A(:,1), A(:,2));
tol = 1e-6;
if min(r(m)) < p.r_min - tol || max(r(m)) > p.r_max + tol || max(vv(m)) > p.v_max + tol || ...
   max(aa(m)) > p.a_max + tol || min(th(m)) < p.theta_min || max(th(m)) > p.theta_max
    error('target_scenario:envelope', ['%s violates the envelope: r %.0f-%.0f m, v <= %.1f m/s, ' ...
        'a <= %.2f m/s^2, theta %.1f to %.1f deg'], sp.name, min(r(m)), max(r(m)), max(vv(m)), ...
        max(aa(m)), min(th(m))*180/pi, max(th(m))*180/pi);
end
bo = zeros(0, 2);  if isfield(sp, 'bo'), bo = sp.bo; end
if isempty(bo), t_boo = Inf; t_bo = 0; else, t_boo = bo(1,1); t_bo = bo(1,2) - bo(1,1); end
nk = floor(T/p.dt_radar) + 2;
rs_ = rng;
rng(p.noise_seed + 7919*sp.id, 'twister');
nz = randn(nk, 2);
rng(rs_);
S = struct('t',t,'x',P(:,1),'y',P(:,2),'vx',V(:,1),'vy',V(:,2),'ax',A(:,1),'ay',A(:,2), ...
           't_boo',t_boo,'t_bo',t_bo,'bo',bo,'id',sp.id,'noise',nz,'name',sp.name);
end

% ---------------------------------------------------------------------------
function [x,y,vx,vy,ax,ay] = integrate_(x0,y0,vx0,vy0,Ax,Ay,wx,wy,fx,fy,t_man,t,h,w)
tau = t - t_man;  s = w(tau);
ax = Ax*s.*sin(wx*max(tau,0) + fx);
ay = Ay*s.*sin(wy*max(tau,0) + fy);
vx = vx0 + cumtrapz(t, ax);  vy = vy0 + cumtrapz(t, ay);
x  = x0 + cumtrapz(t, vx);   y  = y0 + cumtrapz(t, vy);
end

function [ok, info] = trajectory_ok_(x0,y0,vx0,vy0,Ax,Ay,wx,wy,fx,fy,t_man,t,h,w,p)
[x,y,vx,vy,ax,ay] = integrate_(x0,y0,vx0,vy0,Ax,Ay,wx,wy,fx,fy,t_man,t,h,w);
m = t <= p.t_end;  r = hypot(x,y);
th = atan2(y(m), x(m));                       % keep random cases within turret reach
ok = min(r(m)) >= p.r_min+20 && max(r(m)) <= p.r_max-20 && ...
     min(th) >= p.theta_min+5*pi/180 && max(th) <= p.theta_max-5*pi/180 && ...
     max(hypot(vx(m),vy(m))) <= p.v_max-5 && max(hypot(ax(m),ay(m))) <= p.a_max-0.2;
info = [min(r(m)) max(r(m))];
end

function s = smooth_ramp_(tau, T)
% quintic smoothstep 0 -> 1 over T seconds (zero 1st/2nd derivative at ends)
u = min(max(tau/T, 0), 1);
s = u.^3 .* (10 - 15*u + 6*u.^2);
end

function id = scenario_id_(sc)
if ischar(sc) || isstring(sc)
    sc = upper(char(sc));
    switch sc
        case 'A', id = 1;  case 'B', id = 2;  case 'C', id = 3;
        otherwise
            id = str2double(sc);
            if isnan(id), error('Unknown scenario "%s"', sc); end
    end
else
    id = round(double(sc));
end
end

function g = lcg_(seed)
% deterministic uniforms in (0,1), independent of the global RNG
m = 2147483647; a = 16807;
st = mod(abs(seed)*2654435761 + 12345, m); if st == 0, st = 1; end
g = zeros(1,14);
for k = 1:14, st = mod(a*st, m); g(k) = st/m; end
end
