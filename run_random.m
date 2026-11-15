function T = run_random(ids, opts)
%RUN_RANDOM  Generalisation sweep over seeded random bounded cases (ids >= 4).
%   T = run_random(4:28, opts) prints a table and returns per-case metrics.
if nargin < 1, ids = 4:28; end
if nargin < 2, opts = struct(); end
T = zeros(numel(ids), 8);
fprintf('%4s %6s %9s %9s %9s %9s %7s\n','id','hit','t_first','best[m]','rms[m]','blackout','hits');
for q = 1:numel(ids)
    R = run_scenario(ids(q), opts); M = R.metrics; S = target_scenario(ids(q));
    T(q,:) = [ids(q), M.success, M.t_first_hit, M.best_miss, M.rms_err_m, S.t_bo, M.n_hits, M.n_pulses];
    fprintf('%4d %6d %9.2f %9.2f %9.2f %9.2f %4d/%d\n', T(q,:));
end
ok = T(:,2) == 1;
fprintf('success %d/%d | mean first-hit %.2f s | worst %.2f s | mean rms %.2f m | pulses missed %d of %d\n', ...
    sum(ok), numel(ids), mean(T(ok,3)), max(T(ok,3)), mean(T(ok,5)), sum(T(:,8)-T(:,7)), sum(T(:,8)));
end
