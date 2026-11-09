function D = make_test_drones(kind, n_runs, seed)
%MAKE_TEST_DRONES  Synthetic drone paths for estimator testing, within the PDF limits.
%
%   D = make_test_drones(kind, n_runs, seed)
%
%   Every path stays within 800-2000 m range, speed <= 150 m/s, acceleration <= 20 m/s^2
%   and elevation -10 to +85 deg (checked numerically; paths that break a limit are
%   thrown away and redrawn). Turret at the origin, y up, so y < 0 is allowed.
%   Paths run to t_end + t_p + 0.5 s so the truth 0.5 s ahead is always available.
%
%   kind (see list_kinds below):
%     'straight'        constant velocity (Scenario A)
%     'weave_slow'      smooth x-y weave from t = 1 s, w = 0.3-0.8 rad/s (Scenario B style)
%     'weave_med'       same, w = 0.8-1.6 rad/s
%     'weave_fast'      same, w = 1.6-3.0 rad/s
%     'jink'            straight, then a sudden hard turn at 20 m/s^2 (0.2 s onset)
%     'random'          sum of random sines in x and y from t = 0, peak 12-20 m/s^2
%     'near_fast'       800-1050 m, 120-145 m/s crossing, random turning (highest angle rates)
%     'far'             1800-1950 m, medium weave (largest sideways noise)
%     'blackout_C'      medium weave + radar blackout 1.0-1.5 s (Scenario C)
%     'blackout_jink'   jink + 0.5 s blackout starting exactly when the turn starts
%     'blackout_random' any weave + 0.5 s blackout at a random time in 1-9.5 s
%     'blackout_double' random motion + two 0.5 s blackouts (stress case beyond the PDF)
%
%   Output: struct array with fields
%     kind, h (time step, s), t (column), pos (N x 2, m), acc (N x 2), blackouts (k x 2, s)
%
%   Uses its own random stream (seed), so the global rng is untouched.

if nargin < 1
    D = list_kinds();
    return
end
if nargin < 2, n_runs = 30; end
if nargin < 3, seed = 1; end

p = params();
h = 0.005;
T = p.t_end + p.t_p + 0.5;
t = (0:h:T)';
rs = RandStream('mt19937ar', 'Seed', seed);

D = struct('kind', {}, 'h', {}, 't', {}, 'pos', {}, 'acc', {}, 'blackouts', {});
tries = 0;
while numel(D) < n_runs
    tries = tries + 1;
    if tries > 20000
        error('make_test_drones:tooHard', 'Could not draw %s paths within limits.', kind);
    end
    [pos, acc, bo] = one_path(kind, rs, t, h, p);
    if ~within_limits(pos, acc, h, p)
        continue
    end
    k = numel(D) + 1;
    D(k).kind = kind;
    D(k).h = h;
    D(k).t = t;
    D(k).pos = pos;
    D(k).acc = acc;
    D(k).blackouts = bo;
end
end


% =====================================================================
function K = list_kinds()
K = {'straight', 'weave_slow', 'weave_med', 'weave_fast', 'jink', 'random', ...
     'near_fast', 'far', 'blackout_C', 'blackout_jink', 'blackout_random', 'blackout_double'};
end


function [pos, acc, bo] = one_path(kind, rs, t, h, p)
u = @(a, b) a + (b - a) * rand(rs);
N = numel(t);
bo = zeros(0, 2);

% defaults: start point, speed, heading
r0 = u(1000, 1800);
th0 = deg2rad(u(-5, 75));
sp = u(40, 110);
hd = u(0, 2*pi);
mode = 'xy';             % 'xy' = acceleration given in x-y; 'turn' = sideways to velocity
a_xy = zeros(N, 2);
a_turn = zeros(N, 1);

switch kind
    case 'straight'
        sp = u(50, 150);
    case {'weave_slow', 'weave_med', 'weave_fast', 'far', 'blackout_C', 'blackout_random'}
        switch kind
            case 'weave_slow', w = u(0.3, 0.8);
            case 'weave_fast', w = u(1.6, 3.0);
            case 'blackout_random', w = u(0.3, 3.0);
            otherwise, w = u(0.8, 1.6);
        end
        if strcmp(kind, 'far'), r0 = u(1800, 1950); end
        a_xy = weave(t, w, u(10, 20), rs, 1.0);
        if strcmp(kind, 'blackout_C'), bo = [1.0 1.5]; end
        if strcmp(kind, 'blackout_random'), s0 = u(1.0, 9.5); bo = [s0 s0 + 0.5]; end
    case {'jink', 'blackout_jink'}
        mode = 'turn';
        sp = u(70, 140);
        ts = u(1.5, 8.0); dur = u(0.5, 2.0); sgn = sign(rand(rs) - 0.5);
        on = min(max((t - ts) / 0.2, 0), 1) .* min(max((ts + dur - t) / 0.2 + 1, 0), 1);
        a_turn = 20 * sgn * on;
        if strcmp(kind, 'blackout_jink'), bo = [ts ts + 0.5]; end
    case {'random', 'blackout_double'}
        a_xy = random_sines(t, u(12, 20), rs);
        if strcmp(kind, 'blackout_double')
            s1 = u(1.0, 4.0); s2 = s1 + 0.5 + u(1.0, 3.0);
            bo = [s1 s1 + 0.5; s2 s2 + 0.5];
        end
    case 'near_fast'
        mode = 'turn';
        r0 = u(850, 1050);
        sp = u(120, 145);
        hd = th0 + sign(rand(rs) - 0.5) * pi/2 + deg2rad(u(-30, 30));
        a_turn = random_sines(t, u(12, 20), rs);
        a_turn = a_turn(:, 1);
    otherwise
        error('make_test_drones:badKind', 'Unknown kind "%s".', kind);
end

pos = zeros(N, 2);
acc = zeros(N, 2);
pk = r0 * [cos(th0), sin(th0)];
v = sp * [cos(hd), sin(hd)];
for i = 1:N
    if strcmp(mode, 'turn')
        nv = norm(v);
        a = a_turn(i) * [-v(2), v(1)] / max(nv, 1e-9);
    else
        a = a_xy(i, :);
    end
    acc(i, :) = a;
    pos(i, :) = pk;
    v = v + a * h;
    pk = pk + v * h;
end
end


function a = weave(t, w, amp, rs, t_on)
% Smooth weave in x and y; peak |a| <= amp; smooth onset at t_on (no acceleration jump).
wy = w * (0.6 + 0.8 * rand(rs));
ph = 2 * pi * rand(rs, 1, 2);
beta = 2 * pi * rand(rs);
s = smoothstep((t - t_on) / 0.5);
a = amp * s .* [cos(beta) * sin(w * t + ph(1)), sin(beta) * sin(wy * t + ph(2))];
end


function a = random_sines(t, amp, rs)
% Sum of 3 random sines per axis (0.2-3 rad/s), scaled so the peak |a| equals amp.
a = zeros(numel(t), 2);
for ax = 1:2
    for k = 1:3
        w = 0.2 + 2.8 * rand(rs);
        a(:, ax) = a(:, ax) + randn(rs) * sin(w * t + 2 * pi * rand(rs));
    end
end
a = a * amp / max(vecnorm(a, 2, 2));
end


function y = smoothstep(x)
x = min(max(x, 0), 1);
y = x.^2 .* (3 - 2 * x);
end


function ok = within_limits(pos, acc, h, p)
r = vecnorm(pos, 2, 2);
v = diff(pos) / h;
el = atan2(pos(:, 2), pos(:, 1));
ok = all(r >= p.r_min) && all(r <= p.r_max) ...
    && all(vecnorm(v, 2, 2) <= p.v_max + 1e-6) ...
    && all(vecnorm(acc, 2, 2) <= p.a_max + 1e-6) ...
    && all(el >= p.theta_min) && all(el <= p.theta_max);
end
