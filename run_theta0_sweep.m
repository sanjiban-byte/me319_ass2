function T = run_theta0_sweep(scen, th0_list, opts)
%RUN_THETA0_SWEEP  Initial-turret-angle sweep (D21).
%   T = run_theta0_sweep({'A','B','C'}, deg2rad([-10 10 37.5 60 85]))
%   Rows: [scenario index, theta0 deg, success, first hit, n_hits, n_pulses,
%          worst miss, rms err after 2.5 s, peak current]. Prints a table and
%   the worst-case first-hit time.
p = params();
if nargin < 1 || isempty(scen), scen = {'A','B','C'}; end
if nargin < 2 || isempty(th0_list), th0_list = p.theta0_sweep; end
if nargin < 3, opts = struct(); end
if ~iscell(scen), scen = num2cell(scen); end
T = zeros(numel(scen)*numel(th0_list), 9);  q = 0;
fprintf('%5s %8s %4s %8s %7s %9s %8s %7s\n', 'scen', 'th0[deg]', 'hit', 't_first', 'hits', 'worst[m]', 'rms[m]', 'i_pk[A]');
for a = 1:numel(scen)
    for b = 1:numel(th0_list)
        o = opts;  o.th0 = th0_list(b);
        R = run_scenario(scen{a}, o);  M = R.metrics;  L = R.log;
        ix = L.t >= 2.5 & L.th >= p.theta_min & L.th <= p.theta_max;
        q = q + 1;
        T(q,:) = [a, rad2deg(th0_list(b)), M.success, M.t_first_hit, M.n_hits, M.n_pulses, ...
                  max([M.miss, 0]), sqrt(mean(L.err_m(ix).^2)), M.peak_i];
        fprintf('%5s %8.1f %4d %8.2f %3d/%-3d %9.2f %8.2f %7.1f\n', label_(scen{a}), T(q,2), ...
                T(q,3), T(q,4), T(q,5), T(q,6), T(q,7), T(q,8), T(q,9));
    end
end
ok = T(:,3) == 1;
fprintf('intercepted %d/%d | worst first hit %.2f s | pulses missed %d of %d\n', ...
        sum(ok), q, max(T(:,4)), sum(T(:,6) - T(:,5)), sum(T(:,6)));
end
function s = label_(x), if ischar(x), s = x; else, s = sprintf('%d', x); end, end
