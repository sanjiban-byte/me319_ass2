function [specs, th0] = scenario_library(set)
%SCENARIO_LIBRARY  Designed scenarios for the test and validation sets (T2).
%   [specs, th0] = scenario_library('test')   46 cases, 3 new families not
%                                             used in any tuning (train)
%   [specs, th0] = scenario_library('val')    16 validation scenarios in
%                                             4 categories (V1-V4)
%   specs: struct array for target_scenario / run_scenario; th0: turret
%   initial angle per scenario [rad]. Geometry: turret at origin, x
%   downrange, y up; r0/th0 [m, deg] start position, v0/psi0 start speed
%   and heading [m/s, deg from +x]. All trajectories are checked against the
%   assignment envelope by target_scenario (an error if one is violated).
%   Train (what parameters were tuned on) = compare_designs.m: A, B, C x
%   5 initial angles + random cases 4-53 + 1004-1053 (115 cases).
p = params();
amax = 0.99*p.a_max;                        % full a_max, kept just inside the limit
switch lower(set)
    case 'test', [specs, th0] = test_set_(p, amax);
    case 'val',  [specs, th0] = val_set_(p, amax);
    otherwise, error('scenario_library: set must be ''test'' or ''val''');
end
end

% ======================================================================
function [S, th0] = val_set_(p, amax)
S = struct([]);
on = @(t, t1, t2, tr) sstep_(t - t1, tr) - sstep_(t - t2, tr);      % smooth window
z  = @(t) zeros(size(t));
k = 0;
% ---- V1: near the limits of distance and velocity ----
% V1a  max speed, closest approach 820 m (minimum range) at t = 5 s
k = k + 1; S = add_(S, pass_('V1a max-speed pass at 820 m', 900 + k, 820, 40, 150, 5));
% V1b  max speed at long range: 150 m/s pass with closest approach 1700 m
k = k + 1; S = add_(S, pass_('V1b max-speed pass at 1700 m', 900 + k, 1700, 45, 150, 5));
% V1c  max speed orbit at 1950 m (near maximum range), a = v^2/r = 11.5 m/s^2
k = k + 1; S = add_(S, orbit_('V1c max-speed orbit at 1950 m', 900 + k, 1950, 15, 150, 1));
% V1d  max speed and near-max acceleration: orbit at 1150 m, a = 19.0 m/s^2
k = k + 1; S = add_(S, orbit_('V1d max-speed orbit at 1150 m (a = 19 m/s^2)', 900 + k, 1150, 4, 148, 1));
% ---- V2: sudden turns at full a_max (0.2 s onset) ----
base = struct('r0', 1700, 'th0', 25, 'v0', 110, 'psi0', 160);
k = k + 1; S = add_(S, turn_('V2a full-a_max turn at 3 s', 900 + k, base, ...
    @(t) amax*on(t, 3.0, 5.0, 0.2), zeros(0, 2)));
k = k + 1; S = add_(S, turn_('V2b full-a_max turn at blackout onset (3 s)', 900 + k, base, ...
    @(t) amax*on(t, 3.0, 5.0, 0.2), [3.0 3.5]));
k = k + 1; S = add_(S, turn_('V2c turn reversal +a_max to -a_max inside a blackout', 900 + k, base, ...
    @(t) amax*(on(t, 2.0, 3.5, 0.2) - on(t, 3.5, 5.0, 0.2)), [3.4 3.9]));
k = k + 1; S = add_(S, turn_('V2d full-a_max turn at 1 s with blackout 1.0-1.5 s', 900 + k, base, ...
    @(t) -amax*on(t, 1.0, 2.5, 0.2), [1.0 1.5]));
% ---- V3: radar loss at different times, and several losses in one run ----
wv = @(t) weave_(t, 14, 11, 0.8, 1.9, pi/4, 0, 1.0);                % own weave (not Scenario B: B/C are train)
w0 = struct('r0', hypot(1800, 400), 'th0', atan2d(400, 1800), 'v0', hypot(95, 10), 'psi0', atan2d(10, -95));
k = k + 1; S = add_(S, cart_('V3a single loss at 1.0 s (earliest)', 900 + k, w0, wv, [1.0 1.5]));
k = k + 1; S = add_(S, cart_('V3b single loss at 8.5 s (late)', 900 + k, w0, wv, [8.5 9.0]));
k = k + 1; S = add_(S, cart_('V3c two losses (2.0 s, 5.0 s)', 900 + k, w0, wv, [2.0 2.5; 5.0 5.5]));
k = k + 1; S = add_(S, cart_('V3d three losses (1.5, 4.0, 7.0 s)', 900 + k, w0, wv, [1.5 2.0; 4.0 4.5; 7.0 7.5]));
% ---- V4: very close to and very far from the turret ----
k = k + 1; S = add_(S, passw_('V4a close weaving pass, closest 830 m', 900 + k, 830, 50, 80, 5, 10, 1.5, zeros(0, 2)));
k = k + 1; S = add_(S, passw_('V4b close pass, blackout at closest approach', 900 + k, 830, 50, 80, 5, 10, 1.5, [4.75 5.25]));
k = k + 1; S = add_(S, orbitw_('V4c far weaving orbit at 1960 m', 900 + k, 1960, 20, 120, 8, 1.2, zeros(0, 2)));
k = k + 1; S = add_(S, orbitw_('V4d far orbit, blackout 5.0-5.5 s', 900 + k, 1960, 20, 120, 8, 1.2, [5.0 5.5]));
th0 = p.theta0_test*ones(1, numel(S));       % baseline initial angle (D7)
end

% ======================================================================
function [S, th0] = test_set_(p, amax)
% Three families never used for tuning: T1 multi-turn jinks (coordinated
% turns at random times), T2 two-frequency weaves, T3 sustained coordinated
% turns. 16 + 15 + 15 = 46 cases (~2/5 of the 115-case train set). Random
% parameters come from a deterministic generator; draws that leave the
% envelope are discarded. Blackouts: 75 % of cases, one window, onset
% 1.0-8.5 s, duration 0.1-0.5 s. Initial turret angle uniform in -10..85 deg.
S = struct([]);  th0 = [];
on = @(t, t1, t2, tr) sstep_(t - t1, tr) - sstep_(t - t2, tr);
fam = {'T1', 16; 'T2', 15; 'T3', 15};
seed = 5000;
for f = 1:3
    nmade = 0;
    while nmade < fam{f, 2}
        seed = seed + 1;  g = lcg_(seed, 24);
        r0 = 1000 + 900*g(1);  thd = 8 + 55*g(2);  v0 = 60 + 85*g(3);
        psi = thd + 90 + (g(4) > 0.5)*180 + (g(5) - 0.5)*60;      % mostly crossing
        base = struct('r0', r0, 'th0', thd, 'v0', v0, 'psi0', psi);
        if g(6) < 0.75
            t1 = 1.0 + 7.5*g(7);  bo = [t1, t1 + 0.1 + 0.4*g(8)];
        else
            bo = zeros(0, 2);
        end
        nm = sprintf('%s-%02d', fam{f, 1}, nmade + 1);
        switch f
            case 1                                   % three jinks
                ts = sort(1 + 7.5*g(9:11));  du = 0.8 + 1.2*g(12:14);
                sg = sign(g(15:17) - 0.5);  am = (8 + 11.5*g(18:20)).*sg;
                an = @(t) am(1)*on(t, ts(1), ts(1) + du(1), 0.3) + ...
                          am(2)*on(t, ts(2), ts(2) + du(2), 0.3) + am(3)*on(t, ts(3), ts(3) + du(3), 0.3);
                sp = turn_(nm, seed, base, an, bo);
            case 2                                   % two-frequency weave
                A1 = 4 + 6*g(9);  A2 = 4 + 5*g(10);  w1 = 0.3 + 1.0*g(11);  w2 = 1.3 + 1.2*g(12);
                ph = 2*pi*g(13:16);  tm = 1 + 1.5*g(17);
                ac = @(t) twofreq_(t, A1, A2, w1, w2, ph, tm);
                sp = cart_(nm, seed, base, ac, bo);
            case 3                                   % sustained coordinated turn
                a3 = (6 + 13*g(9))*sign(g(10) - 0.5);  tm = 1 + 3*g(11);
                sp = turn_(nm, seed, base, @(t) a3*sstep_(t - tm, 0.5), bo);
        end
        try
            target_scenario(sp);                     % envelope check
        catch
            continue
        end
        S = add_(S, sp);  th0(end + 1) = p.theta_min + (p.theta_max - p.theta_min)*g(21); %#ok<AGROW>
        nmade = nmade + 1;
    end
end
end

% ======================================================================
function S = add_(S, sp)
if isempty(S), S = sp; else, S(end + 1) = orderfields(sp, S(1)); end
end
function sp = blank_(name, id, b, bo)
sp = struct('name', name, 'id', id, 'r0', b.r0, 'th0', b.th0, 'v0', b.v0, 'psi0', b.psi0, ...
            'bo', bo, 'acc_t', [], 'acc_n', [], 'acc_xy', []);
end
function sp = turn_(name, id, b, an, bo)
% coordinated turn: left-normal acceleration an(t), constant speed
sp = blank_(name, id, b, bo);
sp.acc_t = @(t) zeros(size(t));  sp.acc_n = an;
end
function sp = cart_(name, id, b, ac, bo)
% Cartesian acceleration profile ac(t) (N x 2)
sp = blank_(name, id, b, bo);  sp.acc_xy = ac;
end
function sp = pass_(name, id, rc, thc, v, tc)
% straight pass with closest approach rc [m] at elevation thc [deg] at t = tc
c = rc*[cosd(thc), sind(thc)];  u = [-sind(thc), cosd(thc)];
p0 = c - v*tc*u;
b = struct('r0', norm(p0), 'th0', atan2d(p0(2), p0(1)), 'v0', v, 'psi0', atan2d(u(2), u(1)));
sp = turn_(name, id, b, @(t) zeros(size(t)), zeros(0, 2));
end
function sp = passw_(name, id, rc, thc, v, tc, aw, ww, bo)
% pass with a lateral weave of amplitude aw [m/s^2], frequency ww [rad/s]
sp = pass_(name, id, rc, thc, v, tc);
sp.acc_n = @(t) aw*sin(ww*(t - tc));  sp.bo = bo;
end
function sp = orbit_(name, id, r, th, v, dirn)
% circular orbit about the turret, counter-clockwise if dirn = 1
b = struct('r0', r, 'th0', th, 'v0', v, 'psi0', th + dirn*90);
sp = turn_(name, id, b, @(t) dirn*v^2/r*ones(size(t)), zeros(0, 2));
end
function sp = orbitw_(name, id, r, th, v, aw, ww, bo)
sp = orbit_(name, id, r, th, v, 1);
sp.acc_n = @(t) v^2/r + aw*sin(ww*t);  sp.bo = bo;
end
function A = weave_(t, Ax, Ay, wx, wy, fx, fy, tm)
s = sstep_(t - tm, 1.0);  tau = max(t - tm, 0);
A = [Ax*s.*sin(wx*tau + fx), Ay*s.*sin(wy*tau + fy)];
end
function A = twofreq_(t, A1, A2, w1, w2, ph, tm)
s = sstep_(t - tm, 1.0);  tau = max(t - tm, 0);
A = s.*[A1*sin(w1*tau + ph(1)) + A2*sin(w2*tau + ph(2)), ...
        A1*sin(w1*tau + ph(3)) + A2*sin(w2*tau + ph(4))]/sqrt(2);
end
function s = sstep_(tau, T)
u = min(max(tau/T, 0), 1);  s = u.^3.*(10 - 15*u + 6*u.^2);
end
function g = lcg_(seed, n)
% n independent uniforms in (0,1) for this seed; global RNG saved and restored
rs = rng;  rng(seed, 'twister');  g = rand(1, n);  rng(rs);
end
