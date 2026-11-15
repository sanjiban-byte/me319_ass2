function S = compare_designs(opts)
%COMPARE_DESIGNS  Train-G study: P_hit threshold sweep (D39 gate), the post-blackout
%   prefilter reset (D27) on vs off, threshold selection and the full metrics.
%   Results: results/compare_designs.md, results/compare_*.csv, and
%   results/train_metrics.md (metrics_report at the selected threshold).
%   Nothing here is used by turret_controller.
%
%   Threshold selection (worst-case rule, TESTING_VALIDATION_METRICS.md sec. 5),
%   on reset ON, in this order:
%     1. highest worst-type success rate (M1);
%     2. highest worst-run per-shot hit rate (M3);
%     3. lowest mean first-hit time (M2);
%     4. most hits in total.
%
%   S = compare_designs()            full study (A-C x theta0 sweep + 50 random cases)
%   S = compare_designs(struct('ids', 4:13, 'th0', deg2rad(37.5)))   quick look
%
%   Method
%   * The firing gate does not affect the motion (the turret follows the
%     reference, not fire_cmd), so each reset setting is simulated ONCE and
%     every gate variant is evaluated by replaying the real fire_logic.m on
%     the logged estimator/turret signals at the 50 ms grid. A check confirms
%     that the replay reproduces the simulated pulses exactly.
%   * Every variant sees the same trajectories and radar noise.
%   * Gate variants: the D39 rule (valid, |d_hat| < R_eng, P_hit >= threshold),
%     threshold swept.
%   * sigma consistency: the true cross-range error of theta_hat divided by
%     sigma_cross (should be ~N(0,1): 68 % within 1 sigma, 95 % within 2).

if nargin < 1, opts = struct(); end
p0 = params('override');                         % clear any old override
ids  = getf_(opts, 'ids', [4:53, 1004:1053]);   % 1000+k: case k with a full 0.5 s blackout
tag  = getf_(opts, 'tag', 'compare_designs');
th0s = getf_(opts, 'th0', p0.theta0_sweep);
gates = getf_(opts, 'gates', default_gates_());
if ~exist('results', 'dir'), mkdir('results'); end

% ---- test set: A, B, C at every theta0, plus random cases at theta0_test ----
cases = struct('scn', {}, 'th0', {}, 'label', {}, 'group', {});
for s = {'A', 'B', 'C'}
    for th0 = th0s
        cases(end+1) = struct('scn', s{1}, 'th0', th0, ...
            'label', sprintf('%s@%g', s{1}, round(rad2deg(th0)*10)/10), ...
            'group', ['Scenario ' s{1}]); %#ok<AGROW>
    end
end
for id = ids
    if id > 1000, grp = 'Random + 0.5 s blackout'; else, grp = 'Random weave'; end
    cases(end+1) = struct('scn', id, 'th0', p0.theta0_test, 'label', sprintf('%d', id), ...
        'group', grp); %#ok<AGROW>
end
nC = numel(cases);  nG = numel(gates);
fprintf('compare_designs: %d cases x %d reset setting(s), %d gate variants\n', nC, numel(getf_(opts, 'resets', [true false])), nG);

resets = getf_(opts, 'resets', [true false]);
S = struct();
for ir = 1:numel(resets)
    params('override', struct('reinit_on_reacq', resets(ir)));
    logs = cell(nC, 1);  replay_ok = true;
    tic;
    for c = 1:nC
        R = run_scenario(cases(c).scn, struct('th0', cases(c).th0));
        T = target_scenario(cases(c).scn);
        logs{c} = struct('L', R.log, 'pulses', R.pulses, 't_boo', T.t_boo, 't_bo', T.t_bo, ...
                         'scn', cases(c).scn, 'group', cases(c).group, ...
                         'est_key', num2str(cases(c).scn));   % same drone + noise for every theta0
        % replay check with the default gate (whatever params.m sets)
        P = replay_(R.log, params());
        if ~isequal(round(1e3*[P.t]), round(1e3*[R.pulses.t]))
            replay_ok = false;
            fprintf('   replay mismatch in case %s\n', cases(c).label);
        end
    end
    fprintf('reset %d: %d runs in %.0f s, replay check %s\n', resets(ir), nC, toc, ...
            ifelse_(replay_ok, 'PASSED', 'FAILED'));
    % ---- evaluate every gate on these logs ----
    G = struct([]);
    for g = 1:nG
        params('override', merge_(struct('reinit_on_reacq', resets(ir)), gates(g).ovr));
        pg = params();
        per = zeros(nC, 6);    % success, t_first, n_pulses, n_hits, worst miss, pulses in blackout
        miss_all = [];  bo_hits = 0; bo_n = 0;
        for c = 1:nC
            P = replay_(logs{c}.L, pg);
            hit = [P.hit];
            if any(hit), tf = P(find(hit, 1)).t; else, tf = NaN; end
            inbo = [P.t] >= logs{c}.t_boo & [P.t] < logs{c}.t_boo + logs{c}.t_bo;
            bo_n = bo_n + sum(inbo);  bo_hits = bo_hits + sum(hit(inbo));
            per(c, :) = [any(hit), tf, numel(P), sum(hit), max([P.miss, 0]), sum(inbo)];
            miss_all = [miss_all, P.miss]; %#ok<AGROW>
        end
        G(g).label = gates(g).label;  G(g).per = per;
        G(g).success = sum(per(:,1));  G(g).n = nC;
        ok = per(:,1) == 1;
        G(g).t_first_mean = mean(per(ok,2));  G(g).t_first_worst = max(per(ok,2));
        G(g).pulses = sum(per(:,3));  G(g).hits = sum(per(:,4));
        G(g).missed = G(g).pulses - G(g).hits;
        G(g).miss_mean = mean(miss_all);  G(g).miss_p95 = prctile_(miss_all, 95);
        G(g).bo_pulses = bo_n;  G(g).bo_hits = bo_hits;
        G(g).cases_with_miss = sum(per(:,4) < per(:,3));
    end
    S(ir).reset = resets(ir);  S(ir).gates = G;
    S(ir).track = tracking_(logs, cases);      % post-blackout tracking error
    save('-v7', fullfile('results', sprintf('%s_reset%d_logs.mat', tag, resets(ir))), 'logs', 'cases');
    S(ir).cons  = consistency_(logs);          % sigma consistency
end
params('override');

report_(S, cases, gates, tag);

% ---- threshold selection on reset ON (worst-case rule) ----
ion = find([S.reset] == true, 1);
if ~isempty(ion)
    grp = {cases.group};  ug = unique(grp, 'stable');
    K = zeros(nG, 4);
    for g = 1:nG
        per = S(ion).gates(g).per;
        sr = cellfun(@(u) mean(per(strcmp(grp, u), 1)), ug);
        hr = per(:,4) ./ max(1, per(:,3));  hr(per(:,3) == 0) = NaN;
        ok = per(:,1) == 1;
        K(g, :) = [min(sr), min(hr), mean(per(ok, 2)), sum(per(:,4))];
    end
    [~, order] = sortrows([-K(:,1), -K(:,2), K(:,3), -K(:,4)]);
    best = order(1);
    fid = fopen(fullfile('results', [tag '.md']), 'a');
    fprintf(fid, '## Threshold selection (reset ON, worst-case rule)\n\n');
    fprintf(fid, 'Order: 1 worst-type success, 2 worst-run per-shot hit rate, 3 mean first hit, 4 total hits.\n\n');
    fprintf(fid, '| Gate | Worst-type success | Worst-run hit rate | Mean first hit (s) | Total hits |\n|---|---|---|---|---|\n');
    for g = 1:nG
        mark = '';  if g == best, mark = ' **(selected)**'; end
        fprintf(fid, '| %s%s | %.1f %% | %.1f %% | %.3f | %d |\n', gates(g).label, mark, 100*K(g,1), 100*K(g,2), K(g,3), K(g,4));
    end
    fclose(fid);
    fprintf('Selected threshold: %s\n', gates(best).label);
    S(ion).selected = gates(best);
    % ---- full metrics on the train set at the selected threshold ----
    L = load(fullfile('results', sprintf('%s_reset1_logs.mat', tag)), 'logs');
    params('override', gates(best).ovr);
    metrics_report(L.logs, 'train', sprintf('Train-G metrics (115 cases, reset ON, %s)', gates(best).label));
    params('override');
end
end

% ======================================================================
function g = default_gates_()
g = struct('label', {}, 'th', {}, 'ovr', {});
for th = [0.5 0.8 0.9 0.95 0.977 0.99 0.995 0.999]
    g(end+1) = struct('label', sprintf('P_hit >= %g', th), 'th', th, ...
        'ovr', struct('fire_phit_min', th)); %#ok<AGROW>
end
end

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

function T = tracking_(logs, cases)
% Cases with a blackout, split by whether the turret was already locked
% (|err| <= 2 m over the 0.5 s before onset). Per case, in [blackout end, +1.5 s]:
% peak and RMS |turret error|, time until |err| <= 1 m held for 0.2 s, and the
% delay from blackout end to the next hit.
T = struct('label', {{}}, 'locked', [], 'peak', [], 'rms', [], 'settle', [], 'next_hit', []);
for c = 1:numel(logs)
    lg = logs{c};
    if ~isfinite(lg.t_boo) || lg.t_bo <= 0, continue; end
    L = lg.L;  te = lg.t_boo + lg.t_bo;
    pre = L.t >= lg.t_boo - 0.5 & L.t < lg.t_boo;
    locked = any(pre) && all(abs(L.err_m(pre)) <= 2);
    ix = L.t >= te - 1e-9 & L.t <= te + 1.5 & isfinite(L.err_m);
    if ~any(ix), continue; end
    e = abs(L.err_m(ix));  tt = L.t(ix);
    good = e <= 1.0;  st = NaN;
    for j = 1:numel(e)-3
        if all(good(j:j+3)), st = tt(j) - te; break; end
    end
    nh = NaN;
    if ~isempty(lg.pulses)
        pt = [lg.pulses.t];  ph = [lg.pulses.hit];
        k = find(pt >= te - 1e-9 & ph, 1);
        if ~isempty(k), nh = pt(k) - te; end
    end
    T.label{end+1} = cases(c).label;  T.locked(end+1) = locked;
    T.peak(end+1) = max(e);  T.rms(end+1) = sqrt(mean(e.^2));
    T.settle(end+1) = st;  T.next_hit(end+1) = nh;
end
end

function C = consistency_(logs)
z = [];  zc = [];
for c = 1:numel(logs)
    L = logs{c}.L;
    ok = isfinite(L.th_ref) & isfinite(L.sig) & L.sig > 0;
    e = L.r .* sin(L.th_hat - L.th);
    zz = e(ok) ./ L.sig(ok);
    co = L.coast(ok) == 1;
    z = [z, zz(~co)]; zc = [zc, zz(co)]; %#ok<AGROW>
end
C.n = numel(z);  C.in1 = mean(abs(z) <= 1);  C.in2 = mean(abs(z) <= 2);  C.rms = sqrt(mean(z.^2));
C.nc = numel(zc);
if C.nc > 0
    C.in1c = mean(abs(zc) <= 1); C.in2c = mean(abs(zc) <= 2); C.rmsc = sqrt(mean(zc.^2));
else
    C.in1c = NaN; C.in2c = NaN; C.rmsc = NaN;
end
end

function report_(S, cases, gates, tag)
fid = fopen(fullfile('results', [tag '.md']), 'w');
out = @(varargin) fprintf_both_(fid, varargin{:});
out('# Train-G study: P_hit threshold sweep (D39 gate), post-blackout reset on/off\n\n');
nid = [cases(cellfun(@isnumeric, {cases.scn})).scn];
out('%d cases: A, B, C at each theta0 (%d), %d random cases, %d random cases with a forced full 0.5 s blackout.\n\n', ...
    numel(cases), numel(cases) - numel(nid), sum(nid < 1000), sum(nid > 1000));
for ir = 1:numel(S)
    out('## Prefilter reset after blackout (D26): %s\n\n', ifelse_(S(ir).reset, 'ON', 'OFF'));
    out('| gate | intercepted | first hit mean (s) | first hit worst (s) | pulses | missed | cases with a miss | mean miss (m) | p95 miss (m) | pulses in blackout (hits) |\n');
    out('|---|---|---|---|---|---|---|---|---|---|\n');
    for g = 1:numel(S(ir).gates)
        G = S(ir).gates(g);
        out('| %s | %d/%d | %.2f | %.2f | %d | %d | %d | %.2f | %.2f | %d (%d) |\n', G.label, G.success, G.n, ...
            G.t_first_mean, G.t_first_worst, G.pulses, G.missed, G.cases_with_miss, G.miss_mean, G.miss_p95, ...
            G.bo_pulses, G.bo_hits);
    end
    T = S(ir).track;  lk = T.locked == 1;
    out('\nAfter the blackout ends, cases locked before onset (%d of %d with a blackout): ', sum(lk), numel(lk));
    out('peak |err| mean %.2f m, worst %.2f m; RMS mean %.2f m; time to |err| <= 1 m (0.2 s held) mean %.2f s, worst %.2f s, never %d; ', ...
        mean(T.peak(lk)), max(T.peak(lk)), mean(T.rms(lk)), mean_ok_(T.settle(lk)), max_ok_(T.settle(lk)), sum(~isfinite(T.settle(lk))));
    out('blackout end -> next hit mean %.2f s, worst %.2f s.\n', mean_ok_(T.next_hit(lk)), max_ok_(T.next_hit(lk)));
    out('Blackout during the first slew (not locked, %d cases): blackout end -> next hit mean %.2f s, worst %.2f s.\n\n', ...
        sum(~lk), mean_ok_(T.next_hit(~lk)), max_ok_(T.next_hit(~lk)));
    C = S(ir).cons;
    out('sigma consistency (theta_hat error / sigma_cross; ideal 68 %% / 95 %% / 1.00): tracking %.0f %% / %.0f %% / RMS %.2f (%d samples); coasting %.0f %% / %.0f %% / RMS %.2f (%d samples)\n\n', ...
        100*C.in1, 100*C.in2, C.rms, C.n, 100*C.in1c, 100*C.in2c, C.rmsc, C.nc);
    % per-case table for the CSV
    for g = 1:numel(S(ir).gates)
        G = S(ir).gates(g);
        fn = fullfile('results', sprintf('%s_reset%d_gate%d.csv', tag, S(ir).reset, g));
        f2 = fopen(fn, 'w');
        fprintf(f2, '# %s, reset %d\ncase,success,t_first_hit_s,n_pulses,n_hits,worst_miss_m,pulses_in_blackout\n', G.label, S(ir).reset);
        for c = 1:numel(cases)
            fprintf(f2, '%s,%d,%.2f,%d,%d,%.3f,%d\n', cases(c).label, G.per(c,:));
        end
        fclose(f2);
    end
end
fclose(fid);
end

function v = mean_ok_(x)
x = x(isfinite(x)); if isempty(x), v = NaN; else, v = mean(x); end
end
function v = max_ok_(x)
x = x(isfinite(x)); if isempty(x), v = NaN; else, v = max(x); end
end
function fprintf_both_(fid, varargin)
fprintf(varargin{:});  fprintf(fid, varargin{:});
end
function s = range_str_(cases)
ids = [cases(cellfun(@isnumeric, {cases.scn})).scn];
if isempty(ids), s = 'none'; else, s = sprintf('%d-%d', min(ids), max(ids)); end
end
function v = prctile_(x, q)
x = sort(x(:)); if isempty(x), v = NaN; return; end
v = x(max(1, min(numel(x), ceil(q/100*numel(x)))));
end
function s = merge_(a, b)
s = a; f = fieldnames(b); for q = 1:numel(f), s.(f{q}) = b.(f{q}); end
end
function v = getf_(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end
function s = ifelse_(c, a, b)
if c, s = a; else, s = b; end
end
