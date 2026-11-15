function print_metrics(R)
%PRINT_METRICS  One-line-per-quantity summary of a run_scenario result.
M = R.metrics;
fprintf('Scenario %s | pulses %d (hits %d) | first hit %.2f s | best miss %.2f m\n', ...
        scn_(R.scenario), M.n_pulses, M.n_hits, M.t_first_hit, M.best_miss);
fprintf('   lock %.2f s | tracking RMS %.2f m (%.2f mrad), max %.2f m | peak V %.0f V, peak i %.0f A\n', ...
        M.t_lock, M.rms_err_m, M.rms_err_mrad, M.max_err_m, M.peak_V, M.peak_i);
for q = 1:numel(R.pulses)
    fprintf('   pulse %d: t = %.2f s, miss = %.2f m %s\n', q, R.pulses(q).t, R.pulses(q).miss, ...
            ifelse_(R.pulses(q).hit, 'HIT', 'miss'));
end
end
function s = scn_(x)
if isstruct(x), s = x.name; elseif ischar(x), s = x; else, s = sprintf('%d', x); end
end
function s = ifelse_(c, a, b), if c, s = a; else, s = b; end, end
