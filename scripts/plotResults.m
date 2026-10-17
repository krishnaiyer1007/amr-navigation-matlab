%PLOTRESULTS Regenerate the comparison and sensitivity figures (README 14: 7-9)
%   plus figures 1-6 for representative success / failure runs.
%   Requires results/logs/benchmark.mat (run runBenchmark first).
root = setupPaths();
cfg = defaultConfig();
figDir = fullfile(root, 'results', 'figures');
B = load(fullfile(root, 'results', 'logs', 'benchmark.mat'));
R = B.results;
ctrls = {'B0', 'B1', 'P'};
names = {'B0 (A* only)', 'B1 (A* + reactive stop)', 'P (full system)'};
cols = [0.55 0.55 0.55; 0.95 0.6 0.1; 0.15 0.45 0.8];
S = summarizeResults(R);

% --- Figure 7: success and collision rates by controller
f = figure('Color', 'w', 'Position', [100 100 800 450]);
vals = [[S.successRate]' [S.collisionRate]'];
b = bar(vals); grid on;
b(1).FaceColor = [0.2 0.65 0.3]; b(2).FaceColor = [0.85 0.25 0.2];
set(gca, 'XTickLabel', names, 'XTickLabelRotation', 0);
ylabel('Runs (%)'); ylim([0 105]);
legend('Success rate', 'Collision rate', 'Location', 'northwest');
title(sprintf('Success and collision rates by controller (%d paired variants)', numel(R) / 3));
for i = 1:size(vals, 1)
    text(i - 0.15, vals(i,1) + 3, sprintf('%.0f%%', vals(i,1)), 'HorizontalAlignment', 'center');
    text(i + 0.15, vals(i,2) + 3, sprintf('%.0f%%', vals(i,2)), 'HorizontalAlignment', 'center');
end
saveFigure(f, fullfile(figDir, 'fig07_success_collision.png'));

% --- Figure 8: travel time and path length (successful runs only)
f = figure('Color', 'w', 'Position', [100 100 1000 450]);
for m = 1:2
    subplot(1, 2, m); hold on; grid on;
    for c = 1:3
        Rc = R(strcmp({R.controller}, ctrls{c}) & [R.success]);
        if m == 1, y = [Rc.time]; else, y = [Rc.pathLength]; end
        drawBox(c, y, cols(c,:));
    end
    set(gca, 'XTick', 1:3, 'XTickLabel', ctrls); xlim([0.4 3.6]);
    if m == 1, ylabel('Travel time (s)'); title('Travel time (successful runs)');
    else, ylabel('Path length (m)'); title('Path length (successful runs)'); end
    xlabel('Controller');
end
saveFigure(f, fullfile(figDir, 'fig08_time_length.png'));

% --- Figure 8b: paired per-variant outcome matrix
f = figure('Color', 'w', 'Position', [100 100 1000 300]);
nv = numel(R) / 3; M = zeros(3, nv);
code = containers.Map({'success', 'collision', 'stall', 'timeout', 'unreachable'}, {1, 4, 3, 2, 2});
for i = 1:numel(R)
    c = find(strcmp(ctrls, R(i).controller)); v = ceil(i / 3);
    if isKey(code, R(i).reason), M(c, v) = code(R(i).reason); else, M(c, v) = 2; end
end
imagesc(M); colormap(gca, [0.2 0.65 0.3; 0.6 0.6 0.6; 0.95 0.6 0.1; 0.85 0.2 0.2]); caxis([0.5 4.5]);
set(gca, 'YTick', 1:3, 'YTickLabel', ctrls);
xlabel('Scenario variant (paired)'); title('Per-variant outcome: green success, grey timeout/unreachable, orange stall, red collision');
saveFigure(f, fullfile(figDir, 'fig08b_paired_outcomes.png'));

% --- Figure 9: sensitivity plots
if isfield(B, 'sens') && ~isempty(B.sens)
    T = B.sens;
    vars = unique({T.variable}, 'stable');
    f = figure('Color', 'w', 'Position', [100 100 1200 650]);
    for q = 1:numel(vars)
        subplot(2, 3, q); hold on; grid on;
        Tq = T(strcmp({T.variable}, vars{q}));
        lv = unique([Tq.level]);
        sr = zeros(size(lv)); cr = sr; mc = sr;
        for j = 1:numel(lv)
            Tj = Tq([Tq.level] == lv(j));
            sr(j) = 100 * mean([Tj.success]); cr(j) = 100 * mean([Tj.collisions] > 0);
            mc(j) = mean([Tj.minClearance]);
        end
        yyaxis left; plot(lv, sr, 'o-', 'LineWidth', 1.5); plot(lv, cr, 's--', 'LineWidth', 1.2);
        ylabel('Rate (%)'); ylim([0 105]);
        yyaxis right; plot(lv, mc, 'd-.'); ylabel('Mean min clearance (m)');
        xlabel(strrep(vars{q}, '_', ' ')); title(['Sensitivity (P): ' vars{q}], 'Interpreter', 'none');
        if q == 1, legend('Success', 'Collision', 'Min clearance', 'Location', 'southwest'); end
    end
    saveFigure(f, fullfile(figDir, 'fig09_sensitivity.png'));
end

% --- representative runs (figures 1-6): P success and B0 failure in S05
rep = {'S05', 'P'; 'S05', 'B0'; 'S06', 'P'; 'S11', 'P'};
for i = 1:size(rep, 1)
    sc = scenarioDefinitions(rep{i,1});
    [r, lg] = runSimulation(sc, rep{i,2}, cfg);
    figs = plotRun(sc, r, lg, cfg, figDir, sprintf('rep_%s_%s', rep{i,1}, rep{i,2}));
    close(figs);
end
fprintf('Figures written to %s\n', figDir);

function drawBox(x, y, col)
    if isempty(y), text(x, 0, 'no successes', 'HorizontalAlignment', 'center'); return; end
    q = prctile_(y, [25 50 75]); iqr = q(3) - q(1);
    lo = min(y(y >= q(1) - 1.5 * iqr)); hi = max(y(y <= q(3) + 1.5 * iqr));
    w = 0.25;
    patch([x-w x+w x+w x-w], [q(1) q(1) q(3) q(3)], col, 'FaceAlpha', 0.5, 'EdgeColor', col * 0.7);
    plot([x-w x+w], [q(2) q(2)], 'k-', 'LineWidth', 2);
    plot([x x], [hi q(3)], 'k-'); plot([x x], [lo q(1)], 'k-');
    out = y(y < lo | y > hi);
    plot(x * ones(size(out)), out, 'k+');
    plot(x + 0.12 * (rand(size(y)) - 0.5), y, '.', 'Color', col * 0.6);
end

function q = prctile_(y, p)
    y = sort(y(:)); n = numel(y);
    q = zeros(size(p));
    for i = 1:numel(p)
        pos = 1 + (n - 1) * p(i) / 100;
        lo = floor(pos); hi = ceil(pos);
        q(i) = y(lo) + (pos - lo) * (y(hi) - y(lo));
    end
end
