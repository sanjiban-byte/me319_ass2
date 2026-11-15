function R = test_estimator(varargin)
%TEST_ESTIMATOR  Score estimator settings on synthetic drones and print result tables.
%
%   R = test_estimator()                     default sweep: CA and Singer settings, all kinds
%   R = test_estimator('n_runs', 10)         fewer runs per drone type (quick look)
%   R = test_estimator('configs', C)         your own settings: struct array, fields
%                                            label (text) and ovr (struct of est_* values)
%   Other options: 'kinds' (cell of names from make_test_drones), 'seed', 'call_dt' (s),
%                  'radar_style' ('pulse' or 'held'), 'tag' (name of the result files),
%                  'quiet' (true = no printing)
%
%   What is measured, at every controller call from the moment est.valid turns true:
%     ideal turret  the turret if the turret path were exactly 1/(1 + t_p s) (matched, no
%                   voltage limit). Miss = sideways distance from the beam to the drone NOW.
%                   "% on target" = share of time the miss is <= R_eng (2 m), i.e. the
%                   chance that a shot fired at a random moment would hit.
%     theta_hat     sideways error of the current estimate, r_true * sin(theta_hat - theta)
%     forecast      sideways error of theta_pred against the drone 0.5 s later (info only)
%     OLS           least-squares fit theta_hat = a + b * theta_true per drone type
%   Radar noise is the same for every setting (common random numbers), so differences
%   between settings come from the settings, not from luck.
%
%   Results: printed tables, and estimator_tests/results/<tag>.md and .mat

opt = struct('n_runs', 30, 'seed', 1, 'call_dt', 0.005, 'radar_style', 'pulse', ...
             'kinds', {make_test_drones()}, 'configs', [], 'tag', '', 'quiet', false);
for i = 1:2:numel(varargin)
    if ~isfield(opt, varargin{i})
        error('test_estimator:badOption', 'Unknown option "%s".', varargin{i});
    end
    opt.(varargin{i}) = varargin{i+1};
end
if isempty(opt.configs), opt.configs = default_configs(); end
if isempty(opt.tag), opt.tag = ['sweep_' datestr(now, 'yyyymmdd_HHMM')]; end %#ok<TNOW1,DATST>

here = fileparts(mfilename('fullpath'));
addpath(fileparts(here));                 % repo root: estimator_step.m, params.m
p = params();
kinds = opt.kinds;
nK = numel(kinds); nC = numel(opt.configs); nR = opt.n_runs;

% Drones and radar noise are drawn once and reused for every setting.
drones = cell(nK, 1);
for k = 1:nK
    drones{k} = make_test_drones(kinds{k}, nR, opt.seed + 1000 * k);
end

hit_turret = nan(nC, nK, nR);  hit_now = nan(nC, nK, nR);  hit_bo = nan(nC, nK, nR);
p99_turret = nan(nC, nK);      max_turret = nan(nC, nK);   rms_now = nan(nC, nK);
hit_fore = nan(nC, nK);        t_valid = nan(nC, nK, nR);  ols = nan(nC, nK, 3);

t0 = tic;
for c = 1:nC
    estimator_step('config', opt.configs(c).ovr);
    for k = 1:nK
        e_t_all = []; e_n_all = []; e_f_all = []; th_all = []; tt_all = [];
        for r = 1:nR
            rsn = RandStream('mt19937ar', 'Seed', opt.seed + 1000 * k + r);
            out = run_one(drones{k}(r), opt, p, rsn);
            sc = out.scored;
            if ~any(sc), continue; end
            hit_turret(c, k, r) = 100 * mean(abs(out.e_turret(sc)) <= p.R_eng);
            hit_now(c, k, r) = 100 * mean(abs(out.e_now(sc)) <= p.R_eng);
            if any(out.in_bo & sc)
                hit_bo(c, k, r) = 100 * mean(abs(out.e_turret(out.in_bo & sc)) <= p.R_eng);
            end
            t_valid(c, k, r) = out.t_valid;
            e_t_all = [e_t_all; out.e_turret(sc)]; %#ok<AGROW>
            e_n_all = [e_n_all; out.e_now(sc)]; %#ok<AGROW>
            e_f_all = [e_f_all; out.e_fore(sc)]; %#ok<AGROW>
            th_all = [th_all; out.theta_hat(sc)]; %#ok<AGROW>
            tt_all = [tt_all; out.theta_true(sc)]; %#ok<AGROW>
        end
        p99_turret(c, k) = pct(abs(e_t_all), 99);
        max_turret(c, k) = max(abs(e_t_all));
        rms_now(c, k) = sqrt(mean(e_n_all.^2));
        hit_fore(c, k) = 100 * mean(abs(e_f_all) <= p.R_eng);
        X = [ones(size(tt_all)) tt_all];
        b = X \ th_all;
        res = th_all - X * b;
        ols(c, k, :) = [b(1) * 1e6, b(2), 1 - sum(res.^2) / sum((th_all - mean(th_all)).^2)];
    end
    if ~opt.quiet
        fprintf('  [%2d/%2d] %-26s done (%.0f s)\n', c, nC, opt.configs(c).label, toc(t0));
    end
end
estimator_step('config');                 % back to defaults

R = struct('opt', opt, 'kinds', {kinds}, 'labels', {{opt.configs.label}}, ...
    'hit_turret', hit_turret, 'hit_now', hit_now, 'hit_bo', hit_bo, ...
    'p99_turret', p99_turret, 'max_turret', max_turret, 'rms_now', rms_now, ...
    'hit_fore', hit_fore, 't_valid', t_valid, 'ols', ols);
R.summary = summarize(R);
txt = report(R);
if ~opt.quiet, fprintf('%s', txt); end
rdir = fullfile(here, 'results');
if ~exist(rdir, 'dir'), mkdir(rdir); end
fid = fopen(fullfile(rdir, [opt.tag '.md']), 'w');
fprintf(fid, '%s', txt);
fclose(fid);
save(fullfile(rdir, [opt.tag '.mat']), 'R');
end


% =====================================================================
function out = run_one(dr, opt, p, rsn)
dtc = opt.call_dt;
step = round(dtc / dr.h);
if abs(step * dr.h - dtc) > 1e-12
    error('test_estimator:callDt', 'call_dt must be a multiple of %g s.', dr.h);
end
tc = (0:dtc:p.t_end)';
nc = numel(tc);
lead = round(p.t_p / dr.h);
nradar = floor(p.t_end / p.dt_radar) + 1;
nz = randn(rsn, nradar, 2);              % radar noise for this run

th_hat = nan(nc, 1); th_pred = nan(nc, 1); valid = false(nc, 1);
in_bo = false(nc, 1);
r_hold = NaN; th_hold = NaN;
estimator_step('reset');
for i = 1:nc
    t = tc(i);
    idx = (i - 1) * step + 1;
    blind = any(t >= dr.blackouts(:, 1) & t < dr.blackouts(:, 2));
    in_bo(i) = any(t >= dr.blackouts(:, 1) & t < dr.blackouts(:, 2) + p.t_p);  % + recovery
    kr = t / p.dt_radar;
    fresh = abs(kr - round(kr)) < 1e-9 && ~blind;
    if fresh
        q = dr.pos(idx, :);
        j = round(kr) + 1;
        r_m = norm(q) + p.sigma_r * nz(j, 1);
        th_m = atan2(q(2), q(1)) + p.sigma_theta * nz(j, 2);
        r_hold = r_m; th_hold = th_m;
        mv = 1;
    elseif strcmp(opt.radar_style, 'held') && ~blind
        r_m = r_hold; th_m = th_hold; mv = 1;      % caller repeats the last reading
    else
        r_m = 0; th_m = 0; mv = 0;                 % no reading in this call
    end
    est = estimator_step(t, r_m, th_m, mv);
    th_hat(i) = est.theta_hat;
    th_pred(i) = est.theta_pred;
    valid(i) = est.valid;
end

idx = (0:nc - 1)' * step + 1;
q_now = dr.pos(idx, :);
q_fut = dr.pos(idx + lead, :);
r_now = vecnorm(q_now, 2, 2);  th_now = atan2(q_now(:, 2), q_now(:, 1));
r_fut = vecnorm(q_fut, 2, 2);  th_fut = atan2(q_fut(:, 2), q_fut(:, 1));

% Ideal matched turret: y' = (theta_pred - y)/t_p, exact for an input that is linear
% between calls, started with no transient at the first valid estimate.
k0 = find(valid, 1);
y = nan(nc, 1);
if ~isempty(k0)
    tau = p.t_p; a = exp(-dtc / tau); g = (tau / dtc) * (1 - a);
    y(k0) = th_hat(k0);
    for i = k0 + 1:nc
        y(i) = a * y(i-1) + th_pred(i) - a * th_pred(i-1) - g * (th_pred(i) - th_pred(i-1));
    end
end

out.scored = valid;
out.t_valid = NaN;
if ~isempty(k0), out.t_valid = tc(k0); end
out.e_turret = r_now .* sin(y - th_now);
out.e_now = r_now .* sin(th_hat - th_now);
out.e_fore = r_fut .* sin(th_pred - th_fut);
out.theta_hat = th_hat;
out.theta_true = th_now;
out.in_bo = in_bo;
end


% =====================================================================
function C = default_configs()
C = struct('label', {}, 'ovr', {});
for q = [1000 2500 4500 10000 20000]
    C(end+1) = struct('label', sprintf('CA q=%g', q), ...
        'ovr', struct('est_model', 'CA', 'est_q_ca', q)); %#ok<AGROW>
end
for tau = [0.5 1 2]
    for sm = [10 15 20]
        C(end+1) = struct('label', sprintf('Singer tau=%g sig=%g', tau, sm), ...
            'ovr', struct('est_model', 'singer', 'est_tau_a', tau, 'est_sigma_m', sm)); %#ok<AGROW>
    end
end
end


% =====================================================================
function S = summarize(R)
% Per setting: mean over drone types, worst drone type, worst single run.
kind_mean = mean(R.hit_turret, 3, 'omitnan');           % nC x nK
S.mean = mean(kind_mean, 2);
[S.worst_kind, wk] = min(kind_mean, [], 2);
S.worst_kind_name = R.kinds(wk)';
S.worst_run = min(reshape(R.hit_turret, size(R.hit_turret, 1), []), [], 2);
S.p99_worst = max(R.p99_turret, [], 2);
S.t_valid = mean(reshape(R.t_valid, size(R.t_valid, 1), []), 2, 'omitnan');
S.kind_mean = kind_mean;

% Worst-case rule: best CA vs best Singer (best = highest worst-type score, then mean).
isCA = startsWith(R.labels, 'CA')';
S.best_ca = pick_best(S, isCA);
S.best_singer = pick_best(S, ~isCA);
S.verdict = '';
if ~isnan(S.best_ca) && ~isnan(S.best_singer)
    a = reshape(R.hit_turret(S.best_ca, :, :), [], 1);
    b = reshape(R.hit_turret(S.best_singer, :, :), [], 1);
    d = a - b; d = d(~isnan(d));
    se = std(d) / sqrt(numel(d));               % same drones and noise -> paired
    S.diff_mean = mean(d); S.diff_se = se;
    mean_ok = S.mean(S.best_ca) >= S.mean(S.best_singer) - 2 * se;
    worst_ok = S.worst_kind(S.best_ca) > S.worst_kind(S.best_singer);
    S.mean_ok = mean_ok; S.worst_ok = worst_ok;
    if mean_ok && worst_ok
        S.verdict = 'CA';
    else
        S.verdict = 'singer';
    end
end
end


function b = pick_best(S, mask)
b = NaN;
idx = find(mask);
if isempty(idx), return; end
key = S.worst_kind(idx) * 1e3 + S.mean(idx);
[~, j] = max(key);
b = idx(j);
end


% =====================================================================
function txt = report(R)
S = R.summary;
o = R.opt;
short = strrep(strrep(R.kinds, 'blackout_', 'bo_'), 'weave_', 'w_');
L = {};
L{end+1} = sprintf('# Estimator test: %s\n', o.tag);
L{end+1} = sprintf(['%d runs per drone type, call every %g ms, radar style "%s", seed %d. ' ...
    'Same drones and radar noise for every setting.\n'], o.n_runs, 1e3 * o.call_dt, o.radar_style, o.seed);
L{end+1} = sprintf(['\n## 1. Ideal matched turret: %% of time on target (miss <= 2 m), mean over runs\n' ...
    'Scored from the moment est.valid turns true until t = 10 s. "worst type" = lowest drone-type mean; ' ...
    '"worst run" = lowest single run.\n\n']);
L{end+1} = table_md(R.labels, short, S.kind_mean, ...
    {'mean', 'worst type', 'worst run', 'p99 miss (m)', 't valid (s)'}, ...
    [S.mean, S.worst_kind, S.worst_run, S.p99_worst, S.t_valid], {'%.1f', '%.1f', '%.0f', '%.2f', '%.2f'});

L{end+1} = sprintf('\n## 2. During blackout + 0.5 s recovery: %% of time on target (ideal turret)\n\n');
bo = find(startsWith(R.kinds, 'blackout'));
L{end+1} = table_md(R.labels, short(bo), mean(R.hit_bo(:, bo, :), 3, 'omitnan'), {}, [], {});

L{end+1} = sprintf('\n## 3. Current estimate theta_hat: %% of time within 2 m / RMS sideways error (m)\n\n');
L{end+1} = table_md(R.labels, short, mean(R.hit_now, 3, 'omitnan'), {}, [], {});
L{end+1} = sprintf('\nRMS sideways error of theta_hat (m):\n\n');
L{end+1} = table_md(R.labels, short, R.rms_now, {}, [], {}, '%.2f');

L{end+1} = sprintf(['\n## 4. Pure 0.5 s forecast (info only): %% of time theta_pred is within 2 m of ' ...
    'the drone 0.5 s later\n\n']);
L{end+1} = table_md(R.labels, short, R.hit_fore, {}, [], {});

L{end+1} = sprintf('\n## 5. Worst-case rule\n\n');
if isempty(S.verdict)
    L{end+1} = sprintf('Needs at least one CA and one Singer setting.\n');
else
    ca = S.best_ca; sg = S.best_singer;
    L{end+1} = sprintf('- Best CA (by worst type): **%s**: mean %.1f, worst type %.1f (%s), worst run %.0f\n', ...
        R.labels{ca}, S.mean(ca), S.worst_kind(ca), S.worst_kind_name{ca}, S.worst_run(ca));
    L{end+1} = sprintf('- Best Singer (by worst type): **%s**: mean %.1f, worst type %.1f (%s), worst run %.0f\n', ...
        R.labels{sg}, S.mean(sg), S.worst_kind(sg), S.worst_kind_name{sg}, S.worst_run(sg));
    L{end+1} = sprintf('- CA minus Singer, per run: %+.2f points (2 x std. error = %.2f)\n', S.diff_mean, 2 * S.diff_se);
    L{end+1} = sprintf('- CA average not much worse: %s. CA worst type better: %s.\n', yn(S.mean_ok), yn(S.worst_ok));
    L{end+1} = sprintf('- **Rule says: %s** (to be confirmed by the team, and on a fresh seed)\n', S.verdict);
end

L{end+1} = sprintf(['\n## 6. OLS fit theta_hat = a + b * theta_true (best CA and best Singer)\n' ...
    'a in micro-rad, b = slope, R2. A perfect estimator gives a = 0, b = 1, R2 = 1.\n\n']);
for c = unique([S.best_ca, S.best_singer])
    if isnan(c), continue; end
    L{end+1} = sprintf('**%s**\n\n| type | a (urad) | b | R2 |\n|---|---|---|---|\n', R.labels{c}); %#ok<AGROW>
    for k = 1:numel(R.kinds)
        L{end+1} = sprintf('| %s | %.1f | %.6f | %.8f |\n', short{k}, R.ols(c, k, 1), R.ols(c, k, 2), R.ols(c, k, 3)); %#ok<AGROW>
    end
    L{end+1} = newline; %#ok<AGROW>
end
txt = [L{:}];
end


function s = yn(b)
if b, s = 'yes'; else, s = 'no'; end
end


function s = table_md(rows, cols, M, extra_cols, E, efmt, fmt)
if nargin < 7, fmt = '%.0f'; end
hdr = ['| setting | ' strjoin(cols, ' | ')];
sep = ['|---|' repmat('---|', 1, numel(cols))];
if ~isempty(extra_cols)
    hdr = [hdr ' | ' strjoin(extra_cols, ' | ')];
    sep = [sep repmat('---|', 1, numel(extra_cols))];
end
s = [hdr ' |' newline sep newline];
for i = 1:numel(rows)
    line = ['| ' rows{i} ' | ' strjoin(arrayfun(@(v) sprintf(fmt, v), M(i, :), 'UniformOutput', false), ' | ')];
    for j = 1:numel(extra_cols)
        line = [line ' | ' sprintf(efmt{j}, E(i, j))]; %#ok<AGROW>
    end
    s = [s line ' |' newline]; %#ok<AGROW>
end
end


function v = pct(x, q)
% Percentile without the Statistics Toolbox (linear interpolation).
x = sort(x(:));
if isempty(x), v = NaN; return; end
pos = 1 + (numel(x) - 1) * q / 100;
lo = floor(pos); hi = ceil(pos);
v = x(lo) + (pos - lo) * (x(hi) - x(lo));
end
