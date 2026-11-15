% RUN_ALL  Scenarios A-C, initial-angle sweep and random cases. Run from the repo root.
%   1. Scenarios A, B, C at theta0 = 37.5 deg: metrics, figures (figs/), summary.
%   2. Initial-angle sweep (D21) over p.theta0_sweep for A, B, C.
%   3. 25 seeded random bounded-maneuver cases (ids 4-28).
%   4. Prefilter unit test.
%   Tables go to results/*.csv.
p = params();
if ~exist('figs', 'dir'), mkdir('figs'); end
if ~exist('results', 'dir'), mkdir('results'); end
sc = {'A', 'B', 'C'};  rows = zeros(numel(sc), 13);
for q = 1:numel(sc)
    R = run_scenario(sc{q});  print_metrics(R);
    try
        plot_scenario(R, fullfile('figs', ['scenario_' sc{q} '.png']));
    catch err
        warning('run_all:plot', 'figure for %s not saved: %s', sc{q}, err.message);
    end
    save(fullfile('results', ['log_' sc{q} '.mat']), 'R');
    M = R.metrics;
    rows(q,:) = [M.success, M.t_first_hit, M.n_pulses, M.n_hits, M.best_miss, max([M.miss 0]), ...
                 M.rms_err_m, M.rms_err_mrad, M.max_err_m, M.peak_V, M.rms_V, M.peak_i, M.rms_i];
end
write_csv(fullfile('results', 'summary.csv'), ['scenario,success,t_first_hit_s,n_pulses,n_hits,' ...
          'best_miss_m,worst_miss_m,rms_err_m,rms_err_mrad,max_err_m,peak_V,rms_V,peak_i_A,rms_i_A'], rows, sc);

T = run_theta0_sweep(sc, p.theta0_sweep);
write_csv(fullfile('results', 'theta0_sweep.csv'), ['scenario_idx,theta0_deg,success,t_first_hit_s,' ...
          'n_hits,n_pulses,worst_miss_m,rms_err_m,peak_i_A'], T);

T = run_random(4:28);
write_csv(fullfile('results', 'random_cases.csv'), 'id,success,t_first_hit_s,best_miss_m,rms_err_m,blackout_s,n_hits,n_pulses', T);

test_shaper
