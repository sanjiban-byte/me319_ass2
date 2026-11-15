function M = metrics_report(logs, tag, title_str)
%METRICS_REPORT  Engagement and estimator metrics of TESTING_VALIDATION_METRICS.md.
%   M = metrics_report(logs, tag, title_str)
%   logs{c}: struct with fields
%     L      run_scenario log (50 ms grid)
%     scn    scenario (id, 'A'/'B'/'C' or designed struct), for truth acceleration
%     group  scenario-type label (rows of the tables)
%     est_key (optional) identity of the drone path AND radar noise. Runs with the
%            same key give the same estimator output (the estimator never sees
%            the turret), so estimator statistics count each key once. Default:
%            every run is its own key.
%   Pulses are re-derived by replaying fire_logic.m with the CURRENT params()
%   (the gate does not change the motion). Writes results/<tag>_metrics.md and
%   results/<tag>_runs.csv and returns the numbers in M.
%
%   Engagement (per run, then per type and overall)
%     M1 success: at least one hit.          M2 first-hit time (no hit: 10 s penalty).
%     M3 per-shot hit rate, median miss.     M4 consistency: |average - worst|.
%   Estimator, e = r sin(theta_hat - theta) [m] at every valid 50 ms tick
%     E1 bias: per-run mean of e, t-test across runs (runs are independent).
%     E2 e = b0 + b1 a_perp (true sideways acceleration): pooled OLS with
%        SE, t, residual SE, R^2; and a run-level t-test on per-run slopes.
%     E3 calibration: share of |e| <= sigma_cross and <= 2 sigma_cross,
%        tracking vs blackout window (blackout + 0.5 s).
%     E4 RMS e, share |e| <= R_eng, 99th percentile |e|.
%   Base MATLAB only.
p = params();
n = numel(logs);
groups = cellfun(@(g) g.group, logs, 'UniformOutput', false);
keys = cell(n, 1);
for c = 1:n
    if isfield(logs{c}, 'est_key'), keys{c} = logs{c}.est_key; else, keys{c} = sprintf('run%d', c); end
end
[~, first] = unique(keys, 'stable');
est_use = false(n, 1);  est_use(first) = true;      % one estimator sample per drone+noise
[ug, ~, gi] = unique(groups, 'stable');
nG = numel(ug);

R = struct('group', groups(:), 'success', 0, 't_first', NaN, 'pulses', 0, 'hits', 0, ...
    'misses', [], 'bias', NaN, 'slope', NaN, 'e', [], 'ap', [], 'sig', [], 'inbo', []);
R = repmat(R(1), n, 1);
for c = 1:n
    L = logs{c}.L;  R(c).group = groups{c};
    % ---- engagement: replay the gate ----
    P = replay_(L, p);
    hit = [P.hit];
    R(c).pulses = numel(P);  R(c).hits = sum(hit);  R(c).misses = [P.miss];
    R(c).success = any(hit);
    if any(hit), R(c).t_first = P(find(hit, 1)).t; end
    % ---- estimator ----
    ok = isfinite(L.th_ref) & isfinite(L.th_hat) & isfinite(L.sig);
    e  = L.r .* sin(L.th_hat - L.th);
    [~, ~, ~, th, ~, ~, ax, ay] = target_truth(L.t, logs{c}.scn);
    ap = -sin(th).*ax + cos(th).*ay;                   % sideways (across the beam) acceleration
    S = target_scenario(logs{c}.scn);  bo = S.bo;
    inbo = false(size(L.t));
    for k = 1:size(bo, 1), inbo = inbo | (L.t >= bo(k,1) & L.t < bo(k,2) + p.t_p); end
    R(c).e = e(ok);  R(c).ap = ap(ok);  R(c).sig = L.sig(ok);  R(c).inbo = inbo(ok);
    R(c).bias = mean(e(ok));
    if std(ap(ok)) > 0.5                               % slope undefined without sideways acceleration
        b = [ones(sum(ok), 1), ap(ok).'] \ e(ok).';
        R(c).slope = b(2);
    end
end

% ======================= engagement tables =======================
G = struct();
for g = 1:nG + 1
    if g <= nG, ix = find(gi == g); G(g).name = ug{g}; else, ix = (1:n).'; G(g).name = 'All'; end
    X = R(ix);
    G(g).runs = numel(ix);
    G(g).succ = mean([X.success]);
    tf = [X.t_first];  tf(isnan(tf)) = p.t_end;        % no hit: 10 s penalty
    G(g).nohit = sum(~[X.success]);
    G(g).tf_mean = mean(tf);  G(g).tf_med = median(tf);  G(g).tf_worst = max(tf);
    G(g).pulses = sum([X.pulses]);  G(g).hits = sum([X.hits]);
    G(g).hr = G(g).hits / max(1, G(g).pulses);
    hr_run = [X.hits] ./ max(1, [X.pulses]);  hr_run([X.pulses] == 0) = NaN;
    G(g).hr_worst = min(hr_run);
    allm = [X.misses];
    G(g).miss_med = median_(allm);  G(g).miss_worst = max([allm, NaN]);
    mm_run = arrayfun(@(r) median_(r.misses), X);
    % M4, within this type: |mean over runs - worst run|
    G(g).d_tf = abs(mean(tf) - max(tf));
    G(g).d_hr = abs(mean(hr_run, 'omitnan') - min(hr_run));
    G(g).d_mm = abs(mean(mm_run, 'omitnan') - max(mm_run));
    G(g).mm_run_mean = mean(mm_run, 'omitnan');
end
% M4 across types: |average of the type values - worst type|
A = G(1:nG);
X4.succ = abs(mean([A.succ]) - min([A.succ]));
X4.tf   = abs(mean([A.tf_mean]) - max([A.tf_mean]));
X4.hr   = abs(mean([A.hr]) - min([A.hr]));
X4.mm   = abs(mean([A.miss_med]) - max([A.miss_med]));

% ======================= estimator tables =======================
E = struct();
for g = 1:nG + 1
    if g <= nG, ix = find(gi == g & est_use); E(g).name = ug{g}; else, ix = find(est_use); E(g).name = 'All'; end
    X = R(ix);
    % E1: bias across runs (independent runs only)
    bi = [X.bias];  N = numel(bi);
    E(g).b = mean(bi);  E(g).N = N;
    [E(g).t1, E(g).p1, E(g).ci1] = ttest1_(bi);
    % E2: pooled OLS of e on a_perp
    e = [X.e].';  a = [X.ap].';
    if std(a) > 0.5
        Xd = [ones(numel(a), 1), a];  beta = Xd \ e;  res = e - Xd*beta;
        sh = sqrt(sum(res.^2) / (numel(e) - 2));
        se = sh / sqrt(sum((a - mean(a)).^2));
        E(g).b0 = beta(1);  E(g).b1 = beta(2);  E(g).se1 = se;  E(g).tols = beta(2)/se;
        E(g).sres = sh;  E(g).R2 = 1 - sum(res.^2)/sum((e - mean(e)).^2);
    else
        [E(g).b0, E(g).b1, E(g).se1, E(g).tols, E(g).sres, E(g).R2] = deal(NaN);
    end
    sl = [X.slope];  sl = sl(isfinite(sl));
    E(g).Ns = numel(sl);  E(g).b1run = mean_(sl);
    [E(g).tb, E(g).pb] = ttest1_(sl);
    % E3: calibration
    z = [X.e] ./ [X.sig];  ib = logical([X.inbo]);
    E(g).c1 = mean(abs(z(~ib)) <= 1);  E(g).c2 = mean(abs(z(~ib)) <= 2);
    if any(ib), E(g).c1b = mean(abs(z(ib)) <= 1); E(g).c2b = mean(abs(z(ib)) <= 2);
    else, E(g).c1b = NaN; E(g).c2b = NaN; end
    % E4: accuracy
    ee = [X.e];
    E(g).rms = sqrt(mean(ee.^2));  E(g).in2 = mean(abs(ee) <= p.R_eng);  E(g).p99 = pct_(abs(ee), 99);
end

% ======================= write =======================
M = struct('runs', R, 'G', G, 'X4', X4, 'E', E, 'phit_min', p.fire_phit_min);
if ~exist('results', 'dir'), mkdir('results'); end
fid = fopen(fullfile('results', [tag '_metrics.md']), 'w');
w = @(varargin) fprintf(fid, varargin{:});
w('# %s\n\n', title_str);
w(['Metrics as defined in `TESTING_VALIDATION_METRICS.md`. Gate replayed with the current params: ' ...
   'fire when valid, |d_hat| < %.0f m and P_hit >= %g. %d runs. Generated by `metrics_report.m`.\n\n'], ...
   p.R_eng, p.fire_phit_min, n);
w('## Engagement (M1-M3)\n\n');
w('| Type | Runs | M1 success | M2 first hit mean / median / worst (s) | Pulses | Hits | M3 hit rate pooled / worst run | M3 median miss / worst miss (m) |\n');
w('|---|---|---|---|---|---|---|---|\n');
for g = 1:numel(G)
    q = G(g);
    w('| %s | %d | %d/%d (%.0f %%) | %.2f / %.2f / %.2f | %d | %d | %.1f %% / %.1f %% | %.2f / %.2f |\n', ...
        bold_(q.name), q.runs, round(q.succ*q.runs), q.runs, 100*q.succ, q.tf_mean, q.tf_med, q.tf_worst, ...
        q.pulses, q.hits, 100*q.hr, 100*q.hr_worst, q.miss_med, q.miss_worst);
end
w('\nRuns with no hit count as 10 s in M2 (none if M1 is 100 %%).\n\n');
w('## Consistency (M4): |average - worst|, smaller is better\n\n');
w('Within each type (average over runs vs worst run):\n\n');
w('| Type | Δ first hit (s) | Δ hit rate (points) | Δ per-run median miss (m) |\n|---|---|---|---|\n');
for g = 1:numel(G)
    w('| %s | %.2f | %.1f | %.2f |\n', bold_(G(g).name), G(g).d_tf, 100*G(g).d_hr, G(g).d_mm);
end
w('\nAcross types (average of the type values vs worst type): Δ success %.1f points; Δ first hit %.2f s; Δ hit rate %.1f points; Δ median miss %.2f m.\n\n', ...
    100*X4.succ, X4.tf, 100*X4.hr, X4.mm);
w('## Estimator: bias (E1), calibration (E3), accuracy (E4)\n\n');
w(['e = r sin(theta_hat - theta), positive = estimate above the drone. E1 uses one mean per run (runs are independent; ticks inside a run are not). ' ...
   'Runs that share the same drone path and radar noise (one scenario flown from several turret start angles) give identical estimates, ' ...
   'so they count once here: "Runs" = independent estimator runs (%d of %d). With 1 run there is no t-test.\n\n'], sum(est_use), n);
w('| Type | Runs | E1 bias b (m) | 95 %% CI (m) | t (df) | p | E3 within 1σ / 2σ, tracking | E3 within 1σ / 2σ, blackout+0.5 s | E4 RMS (m) | E4 within 2 m | E4 p99 (m) |\n');
w('|---|---|---|---|---|---|---|---|---|---|---|\n');
for g = 1:numel(E)
    q = E(g);
    w('| %s | %d | %+.4f | [%+.4f, %+.4f] | %.2f (%d) | %.3f | %.0f %% / %.0f %% | %s | %.2f | %.1f %% | %.2f |\n', ...
        bold_(q.name), q.N, q.b, q.ci1(1), q.ci1(2), q.t1, q.N - 1, q.p1, 100*q.c1, 100*q.c2, ...
        cov_str_(q.c1b, q.c2b), q.rms, 100*q.in2, q.p99);
end
w('\nIdeal Gaussian coverage: 68.3 %% within 1σ, 95.4 %% within 2σ.\n\n');
w('## Estimator: error vs sideways acceleration (E2)\n\n');
w('e = b0 + b1·a_perp, a_perp = true drone acceleration across the line of sight. b1 in m per (m/s²) = s²: a 20 m/s² turn shifts the estimate by 20·b1 m. The pooled OLS SE treats ticks as independent (too optimistic); the run-level t-test on per-run slopes is the one to use for conclusions.\n\n');
w('| Type | Pooled b1 (s²) | SE(b1) | t_OLS | Residual SE (m) | R² | Run-level mean b1 (s²) | t_β (runs) | p | Shift at 20 m/s² (m) |\n');
w('|---|---|---|---|---|---|---|---|---|---|\n');
for g = 1:numel(E)
    q = E(g);
    w('| %s | %+.5f | %.5f | %.1f | %.3f | %.3f | %+.5f | %.2f (%d) | %.3f | %+.2f |\n', bold_(q.name), q.b1, q.se1, ...
        q.tols, q.sres, q.R2, q.b1run, q.tb, q.Ns, q.pb, 20*q.b1run);
end
w('\nTypes whose drones never accelerate sideways (straight flight) have no slope (NaN).\n');
fclose(fid);

fid = fopen(fullfile('results', [tag '_runs.csv']), 'w');
fprintf(fid, 'run,type,success,t_first_hit_s,pulses,hits,median_miss_m,worst_miss_m,bias_m,slope_s2\n');
for c = 1:n
    fprintf(fid, '%d,%s,%d,%.2f,%d,%d,%.3f,%.3f,%.4f,%.5f\n', c, R(c).group, R(c).success, R(c).t_first, ...
        R(c).pulses, R(c).hits, median_(R(c).misses), max([R(c).misses, NaN]), R(c).bias, R(c).slope);
end
fclose(fid);
end

% ======================================================================
function P = replay_(L, p)
% Re-run fire_logic on the logged 50 ms grid and score pulses like run_scenario.
clear fire_logic
P = struct('t', {}, 'miss', {}, 'hit', {});
prev = 0;  tl = -Inf;
for j = 1:numel(L.t)
    est = struct('valid', isfinite(L.th_ref(j)), 'theta_hat', L.th_hat(j), ...
                 'r_hat', L.r_hat(j), 'sigma_cross', L.sig(j), 'theta_dot_hat', L.thd_hat(j));
    f = fire_logic(L.t(j), est, L.th_turret(j), L.thd_turret(j), j == 1);
    t = L.t(j);
    if f == 1 && prev == 0 && (t - tl) >= p.t_recharge - 1e-9
        tl = t;
        ux = cos(L.th_turret(j)); uy = sin(L.th_turret(j)); s = L.x(j)*ux + L.y(j)*uy;
        if s >= 0, m = abs(L.x(j)*uy - L.y(j)*ux); else, m = hypot(L.x(j), L.y(j)); end
        P(end+1) = struct('t', t, 'miss', m, 'hit', m <= p.R_eng); %#ok<AGROW>
    end
    prev = f;
end
end

function [t, pv, ci] = ttest1_(x)
% One-sample t-test of mean(x) = 0 (base MATLAB: betainc / betaincinv).
x = x(isfinite(x));  N = numel(x);
if N < 2, t = NaN; pv = NaN; ci = [NaN NaN]; return; end
m = mean(x);  se = std(x)/sqrt(N);  df = N - 1;
t = m/se;
pv = betainc(df/(df + t^2), df/2, 0.5);
q = betaincinv(0.05, df/2, 0.5);  tc = sqrt(df*(1 - q)/q);
ci = m + [-1 1]*tc*se;
end

function v = median_(x)
x = x(isfinite(x));  if isempty(x), v = NaN; else, v = median(x); end
end
function v = mean_(x)
x = x(isfinite(x));  if isempty(x), v = NaN; else, v = mean(x); end
end
function v = pct_(x, q)
x = sort(x(:));  if isempty(x), v = NaN; return; end
pos = 1 + (numel(x) - 1)*q/100;  lo = floor(pos);  hi = ceil(pos);
v = x(lo) + (pos - lo)*(x(hi) - x(lo));
end
function s = cov_str_(a, b)
if isnan(a), s = 'n/a'; else, s = sprintf('%.0f %% / %.0f %%', 100*a, 100*b); end
end
function s = bold_(name)
if strcmp(name, 'All'), s = '**All**'; else, s = name; end
end
