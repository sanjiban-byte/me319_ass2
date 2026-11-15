function pair_reset(tag_on, tag_off)
% PAIR_RESET  Case-by-case comparison of the post-blackout prefilter reset (D27)
%   ON vs OFF, from the logs saved by compare_designs (same trajectories and
%   noise in both). Local study script (D14).
%   pair_reset                  uses compare_designs() output (default tag)
%   pair_reset(tagON, tagOFF)   e.g. pair_reset('compare_resetON', 'compare_resetOFF')
%   Usage from the repo root:  addpath('tools'); pair_reset
%   The results folder is found relative to this file, so the current folder
%   does not matter.
if nargin < 1, tag_on = 'compare_designs'; end
if nargin < 2, tag_off = tag_on; end
res = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'results');
A = load(fullfile(res, [tag_on  '_reset1_logs.mat']));
B = load(fullfile(res, [tag_off '_reset0_logs.mat']));
p = params();
rows = [];  lab = {};
for c = 1:numel(A.logs)
    a = A.logs{c};  b = B.logs{c};
    if ~isfinite(a.t_boo) || a.t_bo <= 0, continue; end
    te = a.t_boo + a.t_bo;
    pre = a.L.t >= a.t_boo - 0.5 & a.L.t < a.t_boo;
    locked = any(pre) && all(abs(a.L.err_m(pre)) <= 2);
    w = a.L.t >= te - 1e-9 & a.L.t <= te + 1.5;
    ea = abs(a.L.err_m(w));  eb = abs(b.L.err_m(w));
    % fraction of the window in which a shot would hit (|err| <= 2 m)
    fa = mean(ea <= p.R_eng);  fb = mean(eb <= p.R_eng);
    nh2 = [NaN NaN];  lgs = {a, b};
    for q = 1:2
        if ~isempty(lgs{q}.pulses)
            pt = [lgs{q}.pulses.t];  ph = [lgs{q}.pulses.hit];
            k1 = find(pt >= te - 1e-9 & ph, 1);
            if ~isempty(k1), nh2(q) = pt(k1) - te; end
        end
    end
    nha = nh2(1);  nhb = nh2(2);
    ma = sum(~[a.pulses.hit]);  mb = sum(~[b.pulses.hit]);
    rows(end+1, :) = [locked, max(ea), max(eb), sqrt(mean(ea.^2)), sqrt(mean(eb.^2)), ...
                      fa, fb, nha, nhb, ma, mb, a.t_bo]; %#ok<SAGROW>
    lab{end+1} = A.cases(c).label; %#ok<SAGROW>
end
lk = rows(:,1) == 1;
fprintf('Blackout cases: %d (locked before onset: %d)\n', size(rows,1), sum(lk));
R = rows(lk, :);
fprintf('Locked cases, window = blackout end .. +1.5 s            ON      OFF\n');
fprintf('  peak |err| mean / worst [m]                    %5.2f/%5.2f  %5.2f/%5.2f\n', mean(R(:,2)), max(R(:,2)), mean(R(:,3)), max(R(:,3)));
fprintf('  RMS |err| mean [m]                              %5.2f        %5.2f\n', mean(R(:,4)), mean(R(:,5)));
fprintf('  share of window with |err| <= 2 m, mean / worst  %4.0f%%/%4.0f%%   %4.0f%%/%4.0f%%\n', 100*mean(R(:,6)), 100*min(R(:,6)), 100*mean(R(:,7)), 100*min(R(:,7)));
nh = R(:, 8:9);  ok = all(isfinite(nh), 2);
fprintf('  blackout end -> next hit, mean / worst [s]      %4.2f/%4.2f    %4.2f/%4.2f\n', mean(nh(ok,1)), max(nh(ok,1)), mean(nh(ok,2)), max(nh(ok,2)));
fprintf('  missed pulses (whole run, these cases)          %d          %d\n', sum(R(:,10)), sum(R(:,11)));
d = R(:,3) - R(:,2);
fprintf('  per case: ON has lower peak error in %d, higher in %d, equal in %d of %d\n', sum(d > 1e-3), sum(d < -1e-3), sum(abs(d) <= 1e-3), numel(d));
full = R(:,12) >= 0.5 - 1e-9;
fprintf('  full 0.5 s blackouts only (%d): peak mean ON %.2f m, OFF %.2f m; on-target share ON %.0f%%, OFF %.0f%%\n', ...
    sum(full), mean(R(full,2)), mean(R(full,3)), 100*mean(R(full,6)), 100*mean(R(full,7)));
[~, k] = sort(d, 'descend');  il = find(lk);
fprintf('  largest gains from ON (case: peak ON vs OFF):');
for q = 1:min(5, numel(k)), fprintf('  %s %.2f vs %.2f', lab{il(k(q))}, R(k(q),2), R(k(q),3)); end
fprintf('\n  largest losses from ON:');
for q = numel(k):-1:max(1, numel(k)-2), fprintf('  %s %.2f vs %.2f', lab{il(k(q))}, R(k(q),2), R(k(q),3)); end
fprintf('\n');
end
