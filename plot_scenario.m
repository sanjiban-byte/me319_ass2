function plot_scenario(R, outfile)
%PLOT_SCENARIO  Standard result figure for one run (owner: Harsh, T2).
%   plot_scenario(R) shows it; plot_scenario(R, 'figs/B.png') also saves it.
%   Panels: (1) target range; (2) elevation: true, estimate, prediction,
%   shaped reference, turret; (3) turret pointing error at the target
%   (cross-range, m) with the +/-R_eng hit band and every laser pulse
%   (filled triangle = hit, cross = miss); (4) drive voltage; (5) drive
%   current. V and i are plotted from the plant-step log (R.fine, 1 ms),
%   so current peaks inside a 50 ms control period are not lost. Grey band =
%   radar blackout. Always printed on a white (light) theme for the paper.
L = R.log; p = params(); d = 180/pi;
vis = 'on'; if nargin > 1, vis = 'off'; end
f = figure('Visible', vis, 'Color', 'w', 'Position', [100 100 900 950]);
if isprop(f, 'Theme'), f.Theme = 'light'; end
S = target_scenario(R.scenario);
bo = S.bo;                                     % k x 2 blackout windows
T  = L.t(end);
if isfield(R, 'fine'), Fn = R.fine; else, Fn = struct('t', L.t, 'V', L.V, 'i', L.i); end
cT = [0 0 0]; cE = [0 0.6 0]; cP = [0.75 0 0.75]; cR = [0 0.6 0.8]; cU = [0.85 0.1 0.1];

ax = gobjects(5, 1);
ax(1) = subplot(5,1,1); hold on;
plot(L.t, L.r, '-', 'Color', [0 0.3 0.8], 'LineWidth', 1.2);
ylabel('r [m]'); style_(ax(1), T);
title(sprintf('Scenario %s: range, elevation, pointing error and drive signals', sc_(R.scenario)));

ax(2) = subplot(5,1,2); hold on;
nv = ~isfinite(L.sig) | ~isfinite(L.th_ref);
H = L.th_hat; Pd = L.th_pred; Rf = L.th_ref; H(nv) = NaN; Pd(nv) = NaN; Rf(nv) = NaN;
h = gobjects(5,1);
h(1) = plot(L.t, L.th*d,        '-',  'Color', cT, 'LineWidth', 1.6);
h(2) = plot(L.t, H*d,           '-',  'Color', cE, 'LineWidth', 1.0);
h(3) = plot(L.t, Pd*d,          '--', 'Color', cP, 'LineWidth', 1.0);
h(4) = plot(L.t, Rf*d,          '-',  'Color', cR, 'LineWidth', 1.0);
h(5) = plot(L.t, L.th_turret*d, '-',  'Color', cU, 'LineWidth', 1.2);
ylabel('elevation [deg]'); style_(ax(2), T); shade_(ax(2), bo);
legend(h, {'true', 'estimate', 'predicted \theta_{pred}', 'shaped reference', 'turret'}, ...
       'Location', 'northeast', 'FontSize', 7);

ax(3) = subplot(5,1,3); hold on;
fill([0 T T 0], p.R_eng*[-1 -1 1 1], [0.85 0.95 0.85], 'EdgeColor', 'none');
plot(L.t, L.err_m, '-', 'Color', cU, 'LineWidth', 1.2);
hh = gobjects(0); lab = {};
if ~isempty(R.pulses)
    pt = [R.pulses.t]; hit = [R.pulses.hit]; pm = [R.pulses.miss];
    if any(hit)
        hh(end+1) = plot(pt(hit), zeros(1, sum(hit)), 'v', 'Color', cE, ...
            'MarkerFaceColor', cE, 'MarkerSize', 6);
        lab{end+1} = sprintf('hit (%d)', sum(hit));
    end
    if any(~hit)
        hh(end+1) = plot(pt(~hit), zeros(1, sum(~hit)), 'x', 'Color', 'r', ...
            'MarkerSize', 9, 'LineWidth', 2);
        lab{end+1} = sprintf('miss (%d)', sum(~hit));
    end
    worst = max(pm);
    text(0.99*T, -0.85*8, sprintf('worst pulse miss %.2f m', worst), ...
         'HorizontalAlignment', 'right', 'FontSize', 7);
end
ylim([-8 8]); ylabel('pointing error [m]'); style_(ax(3), T); shade_(ax(3), bo);
if ~isempty(hh), legend(hh, lab, 'Location', 'northeast', 'FontSize', 7); end

ax(4) = subplot(5,1,4); hold on;
plot(Fn.t, Fn.V, '-', 'Color', [0 0.3 0.8], 'LineWidth', 1.0);
plot([0 T], p.V_max*[1 1], 'k:'); plot([0 T], -p.V_max*[1 1], 'k:');
ylim(1.15*p.V_max*[-1 1]); ylabel('V_m [V]'); style_(ax(4), T); shade_(ax(4), bo);

ax(5) = subplot(5,1,5); hold on;
plot(Fn.t, Fn.i, '-', 'Color', cU, 'LineWidth', 1.0);
[ip, k] = max(abs(Fn.i));
plot(Fn.t(k), Fn.i(k), 'ko', 'MarkerSize', 5);
text(Fn.t(k) + 0.15, Fn.i(k), sprintf('peak %.1f A', ip), 'FontSize', 7);
ylabel('i [A]'); xlabel('t [s]'); style_(ax(5), T); shade_(ax(5), bo);
linkaxes(ax, 'x');

if nargin > 1
    if exist('exportgraphics') > 0                     %#ok<EXIST> (MATLAB R2020a+)
        exportgraphics(f, outfile, 'Resolution', 200, 'BackgroundColor', 'white');
    else
        print(f, outfile, '-dpng', '-r200');
    end
    close(f);
end
end

function style_(a, T)
grid(a, 'on'); box(a, 'on'); xlim(a, [0 T]);
set(a, 'FontSize', 8, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');
end
function shade_(a, bo)
yl = ylim(a);
for q = 1:size(bo, 1)
    p = patch(a, [bo(q,1) bo(q,2) bo(q,2) bo(q,1)], [yl(1) yl(1) yl(2) yl(2)], [0.6 0.6 0.6], ...
              'FaceAlpha', 0.25, 'EdgeColor', 'none');
    uistack(p, 'bottom');
end
ylim(a, yl);
end
function s = sc_(x)
if isstruct(x), s = x.name; elseif ischar(x), s = x; else, s = sprintf('%d', x); end
end
