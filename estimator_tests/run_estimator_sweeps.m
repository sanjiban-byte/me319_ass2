%RUN_ESTIMATOR_SWEEPS  Reproduce every estimator result table in estimator_tests/results/.
%
%   Run from MATLAB (any folder):  run('estimator_tests/run_estimator_sweeps.m')
%   Takes about 10 minutes. Writes:
%     results/sweep1_seed1.md           first sweep: 5 CA and 9 Singer settings, seed 1
%     results/sweep2_extended_seed1.md  grid extended past the best values of sweep 1, seed 1
%     results/confirm_seed2.md          best settings re-tested on fresh drones and noise, seed 2
%   (plus a .mat file next to each, with all the numbers)
%
%   These are the results behind the choice est_model = 'singer', est_tau_a = 1,
%   est_sigma_m = 10 (see TUNABLE_PARAMETERS.md).

here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fileparts(here));

% 1. First sweep (default grid inside test_estimator)
test_estimator('tag', 'sweep1_seed1', 'quiet', true);

% 2. Extended grid: lower CA noise, smaller and shorter Singer accelerations
C = struct('label', {}, 'ovr', {});
for q = [100 250 500 1000]
    C(end+1) = ca(q); %#ok<SAGROW>
end
for tau = [0.25 0.5 1]
    for sm = [5 7.5 10]
        C(end+1) = singer(tau, sm); %#ok<SAGROW>
    end
end
test_estimator('configs', C, 'tag', 'sweep2_extended_seed1', 'quiet', true);

% 3. Confirmation on a fresh seed with more runs
C = [ca(100), ca(250), ca(500), singer(0.5, 7.5), singer(0.5, 10), singer(1, 10)];
test_estimator('configs', C, 'seed', 2, 'n_runs', 40, 'tag', 'confirm_seed2', 'quiet', true);

fprintf('Done. Tables are in %s\n', fullfile(here, 'results'));


function c = ca(q)
c = struct('label', sprintf('CA q=%g', q), 'ovr', struct('est_model', 'CA', 'est_q_ca', q));
end


function c = singer(tau, sm)
c = struct('label', sprintf('Singer tau=%g sig=%g', tau, sm), ...
    'ovr', struct('est_model', 'singer', 'est_tau_a', tau, 'est_sigma_m', sm));
end
