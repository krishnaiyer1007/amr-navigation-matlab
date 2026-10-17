%RUNFULLSYSTEM Run the proposed controller P on all 12 nominal scenarios.
%   P: A* + moving-obstacle prediction + DWA-style local planner + replanning.
%   Saves per-run figures (README figures 1-6) for every scenario,
%   results/tables/fullsystem_results.csv and results/logs/fullsystem.mat.
%   Set makeFigures = false to skip plotting.
if ~exist('makeFigures', 'var'), makeFigures = true; end
root = setupPaths();
cfg = defaultConfig();
ids = scenarioDefinitions('list');
results = []; logs = {};
figDir = fullfile(root, 'results', 'figures', 'scenarios');
if ~exist(figDir, 'dir'), mkdir(figDir); end
fprintf('=== Full system (P) ===\n');
for i = 1:numel(ids)
    sc = scenarioDefinitions(ids{i});
    [r, lg] = runSimulation(sc, 'P', cfg);
    ok = strcmp(r.reason, 'success') || strcmp(r.reason, sc.expect);
    fprintf('%s %-12s expected=%-11s %s  time=%6.1f s  minClr=%6.3f m  replans=%d\n', ids{i}, r.reason, ...
        sc.expect, ternaryStr(ok, 'OK  ', 'FAIL'), r.time, r.minClearance, r.replans);
    results = [results r]; %#ok<AGROW>
    logs{end+1} = lg; %#ok<SAGROW>
    if makeFigures
        figs = plotRun(sc, r, lg, cfg, figDir, sprintf('%s_P', ids{i}));
        close(figs);
    end
end
saveResultsTable(results, fullfile(root, 'results', 'tables', 'fullsystem_results.csv'));
printSummary(summarizeResults(results), fullfile(root, 'results', 'tables', 'fullsystem_summary.csv'));
save(fullfile(root, 'results', 'logs', 'fullsystem.mat'), 'results', 'logs', 'cfg');

function s = ternaryStr(c, a, b)
    if c, s = a; else, s = b; end
end
