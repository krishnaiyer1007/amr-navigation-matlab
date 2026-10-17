%RUNSCENARIOS Integration test: all 12 scenarios with P meet their expectation.
%   Expected outcome is 'success' except S07 ('unreachable' = safe stop).
%   Also checks zero collisions across the nominal suite.
root = setupPaths();
cfg = defaultConfig();
ids = scenarioDefinitions('list');
pass = true(1, numel(ids));
results = [];
for i = 1:numel(ids)
    sc = scenarioDefinitions(ids{i});
    r = runSimulation(sc, 'P', cfg);
    results = [results r]; %#ok<AGROW>
    pass(i) = strcmp(r.reason, sc.expect) && r.collisions == 0;
    fprintf('[%s] %s expected %-11s got %-11s minClr %.3f m\n', ...
        statusStr(pass(i)), ids{i}, sc.expect, r.reason, r.minClearance);
end
saveResultsTable(results, fullfile(root, 'results', 'tables', 'scenarios_P.csv'));
fprintf('%d/%d scenarios passed\n', nnz(pass), numel(pass));
assert(all(pass), 'Scenario integration test failed');

function s = statusStr(ok)
    if ok, s = 'PASS'; else, s = 'FAIL'; end
end
