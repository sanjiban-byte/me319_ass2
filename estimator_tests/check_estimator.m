function check_estimator()
%CHECK_ESTIMATOR  Quick correctness checks for estimator_step.m. Prints PASS/FAIL per check.
%
%   1. Noise-free straight flight: the estimate converges to the truth.
%   2. "Held" radar (caller repeats the last reading with meas_valid = 1) gives exactly
%      the same results as "pulse" radar (meas_valid = 1 only at the 20 Hz instants).
%   3. Calling every 50 ms instead of every 5 ms gives a similar current estimate.
%   4. Automatic reset when t goes back to 0 (D12) equals an explicit reset.
%   5. A misspelt tunable name in 'config' is an error.
%   6. Before the first reading all numbers are NaN and valid = false.

here = fileparts(mfilename('fullpath'));
addpath(fileparts(here));
p = params();
allok = true;

% 1 ---------------------------------------------------------------
for model = {'CA', 'singer'}
    estimator_step('config', struct('est_model', model{1}));
    estimator_step('reset');
    v = [-90 40]; q0 = [1200 900];
    worst = 0;
    for t = 0:0.005:p.t_end
        q = q0 + v * t;
        kr = t / p.dt_radar;
        mv = abs(kr - round(kr)) < 1e-9;
        est = estimator_step(t, norm(q), atan2(q(2), q(1)), mv);
        if t >= 2
            worst = max(worst, abs(norm(q) * sin(est.theta_hat - atan2(q(2), q(1)))));
        end
    end
    allok = report(sprintf('1. noise-free straight, %s: worst sideways error after 2 s = %.2g m', ...
        model{1}, worst), worst < 1e-3) && allok;
end

% 2 ---------------------------------------------------------------
cfg = struct('label', 'CA default', 'ovr', struct());
args = {'configs', cfg, 'kinds', {'weave_med', 'blackout_C'}, 'n_runs', 5, 'quiet', true, 'tag', 'check'};
Rp = test_estimator(args{:}, 'radar_style', 'pulse');
Rh = test_estimator(args{:}, 'radar_style', 'held');
d = max(abs(Rp.hit_turret(:) - Rh.hit_turret(:)));
allok = report(sprintf('2. held vs pulse radar: largest difference = %.3g %%-points', d), d < 1e-9) && allok;

% 3 ---------------------------------------------------------------
R50 = test_estimator(args{:}, 'call_dt', 0.05);
d = abs(mean(R50.hit_now(:)) - mean(Rp.hit_now(:)));
allok = report(sprintf(['3. 50 ms vs 5 ms calls: theta_hat on target %.1f %% vs %.1f %% ' ...
    '(info: ideal turret %.1f %% vs %.1f %%)'], mean(R50.hit_now(:)), mean(Rp.hit_now(:)), ...
    mean(R50.hit_turret(:)), mean(Rp.hit_turret(:))), d < 3) && allok;

% 4 ---------------------------------------------------------------
estimator_step('config');
rs = RandStream('mt19937ar', 'Seed', 5);
z = 1500 + randn(rs, 201, 1); a = 0.5 + 3e-4 * randn(rs, 201, 1);
run = @() arrayfun(@(k) getfield(estimator_step((k - 1) * 0.05, z(k), a(k), 1), 'theta_hat'), 1:201); %#ok<GFLD>
estimator_step('reset'); first = run();
second = run();                         % no explicit reset: t jumps back to 0
allok = report('4. automatic reset when t decreases', isequal(first, second)) && allok;

% 5 ---------------------------------------------------------------
ok = false;
try
    estimator_step('config', struct('est_q_typo', 1));
catch err
    ok = strcmp(err.identifier, 'estimator_step:badConfig');
end
estimator_step('config');
allok = report('5. misspelt tunable name is an error', ok) && allok;

% 6 ---------------------------------------------------------------
estimator_step('reset');
est = estimator_step(0, 0, 0, 0);
allok = report('6. no reading yet: NaN outputs, valid = false', ...
    isnan(est.theta_hat) && isnan(est.theta_pred) && ~est.valid) && allok;

if allok
    fprintf('\nALL CHECKS PASSED\n');
else
    fprintf('\nSOME CHECKS FAILED\n');
end
end


function ok = report(msg, ok)
if ok, s = 'PASS'; else, s = 'FAIL'; end
fprintf('[%s] %s\n', s, msg);
end
