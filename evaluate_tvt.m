function evaluate_tvt(opts)
%EVALUATE_TVT  Test and validation runs with the frozen design; writes the report.
%   evaluate_tvt                 full run: 46 test cases + 16 validation
%                                scenarios x 10 radar-noise seeds
%   evaluate_tvt(struct('n_seeds', 3, 'sets', {{'val'}}))   quick look
%   Writes results/TVT_report.md (= docs/tvt_parameters.md + generated
%   sections 2-4), results/tvt_test_cases.csv, results/tvt_val_runs.csv,
%   results/tvt_tstat_ticks.csv and figures/tvt_tstat.png.
%   No parameter is changed: everything uses params.m as frozen (D31, D27).
%
%   Metrics (TESTING_VALIDATION_METRICS.md, via metrics_report.m):
%   engagement M1-M4 and estimator E1-E4, written to results/test_metrics.md
%   and results/val_metrics.md. This report adds the per-scenario tables and,
%   for validation, a descriptive per-tick t-statistic across the noise seeds.
%   (The earlier OLS theta_hat = a + b*theta_true table was removed: a and b
%   are confounded over the narrow angle range of one scenario, D40.)

if nargin < 1, opts = struct(); end
nS   = getf_(opts, 'n_seeds', 10);
sets = getf_(opts, 'sets', {'test', 'val'});
here = fileparts(mfilename('fullpath'));
if ~exist(fullfile(here, 'results'), 'dir'), mkdir(fullfile(here, 'results')); end
if ~exist(fullfile(here, 'figures'), 'dir'), mkdir(fullfile(here, 'figures')); end
params('override');                                  % frozen parameters only
p = params();
out = {};                                            % report lines

% ======================= TEST =======================
if any(strcmp(sets, 'test'))
    [TS, th0] = scenario_library('test');
    nT = numel(TS);  M = zeros(nT, 9);  fam = cell(nT, 1);  TL = cell(nT, 1);
    fnames = struct('T1', 'T1 multi-jink', 'T2', 'T2 two-frequency weave', 'T3', 'T3 coordinated turn');
    tic;
    for c = 1:nT
        R = run_scenario(TS(c), struct('th0', th0(c)));
        M(c, :) = run_metrics_(R, TS(c));
        fam{c} = TS(c).name(1:2);
        TL{c} = struct('L', R.log, 'scn', TS(c), 'group', fnames.(fam{c}), 'est_key', TS(c).name);
    end
    metrics_report(TL, 'test', 'Test set metrics (46 unseen cases, frozen design)');
    fprintf('test: %d cases in %.0f s\n', nT, toc);
    fid = fopen(fullfile(here, 'results', 'tvt_test_cases.csv'), 'w');
    fprintf(fid, 'case,theta0_deg,blackout_s,intercepted,t_first_hit_s,pulses,hits,mean_miss_m,worst_miss_m,rms_track_m\n');
    for c = 1:nT
        fprintf(fid, '%s,%.1f,%.2f,%d,%.2f,%d,%d,%.3f,%.3f,%.3f\n', TS(c).name, th0(c)*180/pi, M(c,9), M(c,1:7));
    end
    fclose(fid);
    out{end+1} = '## 2. Test set (46 unseen cases, frozen design)';
    out{end+1} = '';
    out{end+1} = '| Family | Cases | Intercepted | First hit mean / worst (s) | Pulses | Hits | Hit accuracy | Mean / worst miss (m) | Tracking RMS (m) |';
    out{end+1} = '|---|---|---|---|---|---|---|---|---|';
    fams = {'T1', 'T1 multi-jink'; 'T2', 'T2 two-frequency weave'; 'T3', 'T3 coordinated turn'; '', 'All test'};
    for f = 1:size(fams, 1)
        if isempty(fams{f,1}), ix = true(nT, 1); else, ix = strcmp(fam, fams{f,1}); end
        out{end+1} = family_row_(fams{f,2}, M(ix, :)); %#ok<AGROW>
    end
    out{end+1} = '';
    out{end+1} = 'Per-case results: `results/tvt_test_cases.csv`.';
    out{end+1} = '';
end

% ======================= VALIDATION =======================
if any(strcmp(sets, 'val'))
    [VS, th0v] = scenario_library('val');
    nV = numel(VS);
    RM = zeros(nV, nS, 9);                            % run metrics
    TT = cell(nV, 1);  VL = {};
    cname = struct('V1', 'V1 near distance / speed limits', 'V2', 'V2 full-a_max turns', ...
                   'V3', 'V3 radar losses', 'V4', 'V4 very close / very far');
    tic;
    for v = 1:nV
        E = []; tk = [];
        for s = 1:nS
            params('override', struct('noise_seed', 3000 + s));
            R = run_scenario(VS(v), struct('th0', th0v(v)));
            RM(v, s, :) = run_metrics_(R, VS(v));
            L = R.log;  ok = isfinite(L.th_ref) & isfinite(L.th_hat);
            VL{end+1} = struct('L', L, 'scn', VS(v), 'group', cname.(VS(v).name(1:2)), ... %#ok<AGROW>
                'est_key', sprintf('%s#%d', VS(v).name, 3000 + s));
            e = L.r.*sin(L.th_hat - L.th);  e(~ok) = NaN;
            E = [E; e]; tk = L.t; %#ok<AGROW>
        end
        Sv = target_scenario(VS(v));
        TT{v} = tick_t_(E, tk, Sv.bo);
        fprintf('  %-55s done\n', VS(v).name);
    end
    params('override');
    fprintf('validation: %d scenarios x %d seeds in %.0f s\n', nV, nS, toc);
    metrics_report(VL, 'val', sprintf('Validation metrics (16 scenarios x %d noise seeds)', nS));
    crit = TT{1}.crit;

    % CSVs
    fid = fopen(fullfile(here, 'results', 'tvt_val_runs.csv'), 'w');
    fprintf(fid, 'scenario,seed,intercepted,t_first_hit_s,pulses,hits,mean_miss_m,worst_miss_m,rms_track_m\n');
    for v = 1:nV, for s = 1:nS
        fprintf(fid, '"%s",%d,%d,%.2f,%d,%d,%.3f,%.3f,%.3f\n', VS(v).name, 3000 + s, squeeze(RM(v, s, 1:7)));
    end, end
    fclose(fid);
    fid = fopen(fullfile(here, 'results', 'tvt_tstat_ticks.csv'), 'w');
    fprintf(fid, 'scenario,t_s,mean_err_m,t_stat,in_blackout_or_0.5s_after\n');
    for v = 1:nV
        for k = 1:numel(TT{v}.t)
            if isfinite(TT{v}.tstat(k))
                fprintf(fid, '"%s",%.2f,%.4f,%.3f,%d\n', VS(v).name, TT{v}.t(k), TT{v}.mu(k), TT{v}.tstat(k), TT{v}.inbo(k));
            end
        end
    end
    fclose(fid);

    % ---- section 3: interception, first hit, hit accuracy across seeds ----
    out{end+1} = sprintf('## 3. Validation: interception, first-hit time and hit accuracy (%d seeds per scenario)', nS);
    out{end+1} = '';
    out{end+1} = 'Hit accuracy = hits / pulses within one run; "mean / min" are over the seeds.';
    out{end+1} = '';
    out{end+1} = '| Scenario | Intercepted | First hit mean +/- sd (s) | First hit worst (s) | Pulses per run | Hit accuracy mean / min | Miss mean +/- sd (m) | Worst miss (m) | Tracking RMS (m) |';
    out{end+1} = '|---|---|---|---|---|---|---|---|---|';
    for v = 1:nV
        out{end+1} = val_row_(VS(v).name, squeeze(RM(v, :, :))); %#ok<AGROW>
    end
    out{end+1} = '';
    cats = {'V1', 'V2', 'V3', 'V4'};
    out{end+1} = '| Category | Runs | Intercepted | First hit mean / worst (s) | Pulses | Missed | Hit accuracy |';
    out{end+1} = '|---|---|---|---|---|---|---|';
    allR = [];
    for c = 1:4
        ix = find(strncmp({VS.name}, cats{c}, 2));
        X = reshape(RM(ix, :, :), [], 9);  allR = [allR; X]; %#ok<AGROW>
        out{end+1} = cat_row_(cats{c}, X); %#ok<AGROW>
    end
    out{end+1} = cat_row_('All validation', allR);
    out{end+1} = '';

    % ---- section 4: estimator ----
    out{end+1} = '## 4. Validation: estimator';
    out{end+1} = '';
    out{end+1} = ['Bias (E1, t-test across runs), error vs sideways acceleration (E2), calibration (E3) and ' ...
                  'accuracy (E4) per category: `results/val_metrics.md` (test set: `results/test_metrics.md`).'];
    out{end+1} = '';
    out{end+1} = sprintf(['Per-tick t-statistic: at every 50 ms tick, t = mean / (sd / sqrt(%d)) of the cross-range ' ...
        'estimate error r(theta_hat - theta) across the %d seeds (H0: zero mean, df = %d, two-sided 5 %% critical ' ...
        'value %.3f). If the estimator were unbiased at every tick, about 5 %% of ticks would exceed it by chance. ' ...
        'The trajectory is the same for every seed, so a deterministic lag (for example during a turn) shows up ' ...
        'as large |t|.'], nS, nS, nS - 1, crit);
    out{end+1} = '';
    out{end+1} = '| Scenario | Ticks | Ticks with \|t\| > crit: all / blackout+0.5 s / other | Max \|t\| (at t) | Mean error (m) |';
    out{end+1} = '|---|---|---|---|---|';
    for v = 1:nV
        q = TT{v};
        out{end+1} = sprintf('| %s | %d | %.0f %% / %s / %.0f %% | %.1f (%.2f s) | %+.3f |', VS(v).name, q.n, ...
            100*q.frac, pct_(q.frac_bo), 100*q.frac_out, q.tmax, q.t_at, q.mean_err); %#ok<AGROW>
    end
    out{end+1} = '';
    out{end+1} = 'Per-tick values: `results/tvt_tstat_ticks.csv`. Figure: `figures/tvt_tstat.png`.';
    out{end+1} = '';
    try
        plot_tstat_(VS, TT, crit, fullfile(here, 'figures', 'tvt_tstat.png'));
    catch err
        warning('evaluate_tvt:plot', 'figure not saved: %s', err.message);
    end
end

% ======================= write report =======================
hdr = fileread(fullfile(here, 'docs', 'tvt_parameters.md'));
fid = fopen(fullfile(here, 'results', 'TVT_report.md'), 'w');
fprintf(fid, '%s\n', hdr);
fprintf(fid, '%s\n', out{:});
fprintf(fid, '\n_Generated by evaluate_tvt.m on %s (%d validation seeds)._\n', datestr(now, 'yyyy-mm-dd HH:MM'), nS); %#ok<TNOW1,DATST>
fclose(fid);
fprintf('Report written to %s\n', fullfile(here, 'results', 'TVT_report.md'));
end

% ======================================================================
function m = run_metrics_(R, sp)
% [intercepted, t_first, pulses, hits, mean miss, worst miss, rms track, -, blackout total]
M = R.metrics;  L = R.log;
ix = L.t >= 2.5 & isfinite(L.err_m);
rmsT = sqrt(mean(L.err_m(ix).^2));
Sp = target_scenario(sp);  bo = Sp.bo;  btot = sum(bo(:,2) - bo(:,1));
mm = M.miss;  if isempty(mm), mm = NaN; end
m = [M.success, M.t_first_hit, M.n_pulses, M.n_hits, mean(mm), max(mm), rmsT, 0, btot];
end

function s = family_row_(name, X)
ok = X(:,1) == 1;
s = sprintf('| %s | %d | %d/%d | %.2f / %.2f | %d | %d | %.1f %% | %.2f / %.2f | %.2f |', name, size(X,1), ...
    sum(ok), size(X,1), mean(X(ok,2)), max(X(ok,2)), sum(X(:,3)), sum(X(:,4)), 100*sum(X(:,4))/max(1,sum(X(:,3))), ...
    nmean_(X(:,5)), max(X(:,6)), mean(X(:,7)));
end

function s = val_row_(name, X)
ok = X(:,1) == 1;  acc = X(:,4)./max(1, X(:,3));
s = sprintf('| %s | %d/%d | %.2f +/- %.2f | %.2f | %.1f | %.1f %% / %.1f %% | %.2f +/- %.2f | %.2f | %.2f |', ...
    name, sum(ok), size(X,1), mean(X(ok,2)), std(X(ok,2)), max(X(ok,2)), mean(X(:,3)), ...
    100*mean(acc), 100*min(acc), nmean_(X(:,5)), nstd_(X(:,5)), max(X(:,6)), mean(X(:,7)));
end

function s = cat_row_(name, X)
ok = X(:,1) == 1;
s = sprintf('| %s | %d | %d/%d | %.2f / %.2f | %d | %d | %.1f %% |', name, size(X,1), sum(ok), size(X,1), ...
    mean(X(ok,2)), max(X(ok,2)), sum(X(:,3)), sum(X(:,3)) - sum(X(:,4)), 100*sum(X(:,4))/max(1, sum(X(:,3))));
end

function o = ols_(d, Lag)
% OLS y = a + b x with ordinary and Newey-West (within-seed) standard errors.
X = [ones(numel(d.x), 1), d.x];  y = d.y;
XtXi = inv(X.'*X);  beta = XtXi*(X.'*y);  u = y - X*beta;
n = numel(y);  s2 = (u.'*u)/(n - 2);  V = s2*XtXi;
S = zeros(2);
for g = unique(d.g).'
    ix = find(d.g == g);  Z = X(ix, :).*u(ix);
    S = S + Z.'*Z;
    for l = 1:Lag
        w = 1 - l/(Lag + 1);
        G = Z(1+l:end, :).'*Z(1:end-l, :);
        S = S + w*(G + G.');
    end
end
Vh = XtXi*S*XtXi;
o.n = n;  o.a = beta(1);  o.b = beta(2);
o.R2 = 1 - (u.'*u)/sum((y - mean(y)).^2);
o.ta = beta(1)/sqrt(V(1,1));         o.tb = (beta(2) - 1)/sqrt(V(2,2));
o.ta_hac = beta(1)/sqrt(Vh(1,1));    o.tb_hac = (beta(2) - 1)/sqrt(Vh(2,2));
end

function q = tick_t_(E, t, bo)
% Per-tick t-statistic of the mean error across seeds (rows of E).
nS = size(E, 1);  ok = all(isfinite(E), 1);
mu = mean(E, 1);  sd = std(E, 0, 1);
ts = mu./(sd/sqrt(nS));  ts(~ok) = NaN;
df = nS - 1;  x = betaincinv(0.05, df/2, 0.5);  q.crit = sqrt(df*(1 - x)/x);
inbo = false(size(t));
for k = 1:size(bo, 1), inbo = inbo | (t >= bo(k,1) & t < bo(k,2) + 0.5); end
q.t = t;  q.mu = mu;  q.tstat = ts;  q.inbo = inbo;  q.n = sum(ok);
big = abs(ts) > q.crit;
q.frac = sum(big & ok)/max(1, sum(ok));
q.frac_bo = sum(big & ok & inbo)/max(1, sum(ok & inbo));
if ~any(ok & inbo), q.frac_bo = NaN; end
q.frac_out = sum(big & ok & ~inbo)/max(1, sum(ok & ~inbo));
[q.tmax, k] = max(abs(ts).*ok);  q.t_at = t(k);
q.mean_err = mean(mu(ok));  Eo = E(:, ok);  q.rms = sqrt(mean(Eo(:).^2));
end

function m = nmean_(x)
x = x(~isnan(x));  if isempty(x), m = NaN; else, m = mean(x); end
end
function m = nstd_(x)
x = x(~isnan(x));  if numel(x) < 2, m = 0; else, m = std(x); end
end

function s = pct_(x)
if isnan(x), s = 'n/a'; else, s = sprintf('%.0f %%', 100*x); end
end

function plot_tstat_(VS, TT, crit, fn)
f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 900 800]);
if isprop(f, 'Theme'), f.Theme = 'light'; end
cats = {'V1', 'V2', 'V3', 'V4'};
ttl = {'V1 near distance / velocity limits', 'V2 sudden full-a_{max} turns', ...
       'V3 radar losses (single and multiple)', 'V4 very close / very far'};
for c = 1:4
    ax = subplot(4, 1, c); hold on; grid on; box on;
    ix = find(strncmp({VS.name}, cats{c}, 2));
    h = gobjects(0); lab = {};
    for v = ix
        h(end+1) = plot(TT{v}.t, TT{v}.tstat, 'LineWidth', 1.0); %#ok<AGROW>
        lab{end+1} = VS(v).name(1:3); %#ok<AGROW>
    end
    plot([0 10], crit*[1 1], 'k:'); plot([0 10], -crit*[1 1], 'k:');
    xlim([0 10]); ylabel('t (per tick)'); title(ttl{c}, 'FontSize', 9);
    set(ax, 'FontSize', 8, 'Color', 'w');
    legend(h, lab, 'Location', 'eastoutside', 'FontSize', 7);
end
xlabel('t [s]');
if exist('exportgraphics') > 0 %#ok<EXIST>
    exportgraphics(f, fn, 'Resolution', 200, 'BackgroundColor', 'white');
else
    print(f, fn, '-dpng', '-r150');
end
close(f);
end

function v = getf_(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end
