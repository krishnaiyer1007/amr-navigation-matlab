%RUNBASELINE Run the two baseline controllers on all 12 nominal scenarios.
%   B0: A* + waypoint tracking (no local avoidance, no replanning)
%   B1: A* + proximity-based stop (reactive safety baseline)
%   Saves results/tables/baseline_results.csv and results/logs/baseline.mat
root = setupPaths();
cfg = defaultConfig();
ids = scenarioDefinitions('list');
ctrls = {'B0', 'B1'};
results = [];
logs = {};
fprintf('=== Baseline controllers (%s) ===\n', strjoin(ctrls, ', '));
for c = 1:numel(ctrls)
    for i = 1:numel(ids)
        sc = scenarioDefinitions(ids{i});
        [r, lg] = runSimulation(sc, ctrls{c}, cfg);
        fprintf('%s %-3s %-12s time=%6.1f s  minClr=%6.3f m\n', ids{i}, ctrls{c}, r.reason, r.time, r.minClearance);
        results = [results r]; %#ok<AGROW>
        logs{end+1} = lg; %#ok<SAGROW>
    end
end
saveResultsTable(results, fullfile(root, 'results', 'tables', 'baseline_results.csv'));
printSummary(summarizeResults(results), fullfile(root, 'results', 'tables', 'baseline_summary.csv'));
save(fullfile(root, 'results', 'logs', 'baseline.mat'), 'results', 'logs', 'cfg');
