%RUNBENCHMARK Paired comparison of B0, B1 and P plus sensitivity sweeps.
%   Main comparison: 30 seeded randomised variants (10 representative
%   scenarios x 3 seeds), each run with all three controllers on the SAME
%   scenario and noise seed (90 paired runs).
%   Sensitivity: controller P, one parameter varied at a time (README 15)
%   on nSens dynamic variants per level.
%   Outputs: results/tables/benchmark_results.csv, benchmark_summary.csv,
%            sensitivity_results.csv, results/logs/benchmark.mat
%   Set quick = true before running for a short smoke test.
if ~exist('quick', 'var'), quick = false; end
if ~exist('doSensitivity', 'var'), doSensitivity = true; end
root = setupPaths();
cfg = defaultConfig();
tStart = tic;

benchIds = {'S02', 'S03', 'S04', 'S05', 'S06', 'S08', 'S09', 'S10', 'S11', 'S12'};
seeds = 1:3;
if quick, benchIds = benchIds(1:4); seeds = 1; end
ctrls = {'B0', 'B1', 'P'};
variants = {};
for s = seeds
    for i = 1:numel(benchIds), variants(end+1, :) = {benchIds{i}, 100 * s + i}; end %#ok<SAGROW>
end
fprintf('=== Benchmark: %d variants x %d controllers ===\n', size(variants, 1), numel(ctrls));
results = [];
for v = 1:size(variants, 1)
    sc = scenarioDefinitions(variants{v, 1}, variants{v, 2});
    line = sprintf('%-3d %s seed %4d:', v, sc.id, sc.seed);
    for c = 1:numel(ctrls)
        r = runSimulation(sc, ctrls{c}, cfg);
        results = [results r]; %#ok<AGROW>
        line = sprintf('%s  %s=%-11s', line, ctrls{c}, r.reason);
    end
    fprintf('%s\n', line);
end
saveResultsTable(results, fullfile(root, 'results', 'tables', 'benchmark_results.csv'));
S = summarizeResults(results);
printSummary(S, fullfile(root, 'results', 'tables', 'benchmark_summary.csv'));

% ---------------- sensitivity sweeps (controller P) ----------------
sens = [];
if doSensitivity
    sweeps = { ...
        'sensor.rangeNoiseStd', [0 0.02 0.05 0.10]; ...
        'obstacleSpeed',        [0 0.3 0.6 1.0]; ...
        'robot.vMax',           [0.3 0.5 0.65]; ...
        'safety.margin',        [0.10 0.15 0.25]; ...
        'sensor.missProb',      [0 0.05 0.15]};
    sensIds = {'S04', 'S05', 'S11', 'S12', 'S06'};
    nSens = 2;  if quick, nSens = 1; sensIds = sensIds(1:2); end
    fprintf('\n=== Sensitivity sweeps (P) ===\n');
    for q = 1:size(sweeps, 1)
        name = sweeps{q, 1}; levels = sweeps{q, 2};
        for lv = levels
            nOk = 0; nRun = 0;
            for i = 1:numel(sensIds)
                for s = 1:nSens
                    sc = scenarioDefinitions(sensIds{i}, 500 + 10 * s + i);
                    c2 = cfg;
                    if strcmp(name, 'obstacleSpeed')
                        for d = 1:numel(sc.dyn), sc.dyn(d).speed = lv; end
                    else
                        parts = strsplit(name, '.');
                        c2.(parts{1}).(parts{2}) = lv;
                        % scenario overrides must not mask the swept value
                        if isfield(sc.overrides, parts{1}) && isfield(sc.overrides.(parts{1}), parts{2})
                            sc.overrides.(parts{1}) = rmfield(sc.overrides.(parts{1}), parts{2});
                        end
                    end
                    r = runSimulation(sc, 'P', c2);
                    r.variable = name; r.level = lv;
                    sens = [sens r]; %#ok<AGROW>
                    nOk = nOk + r.success; nRun = nRun + 1;
                end
            end
            fprintf('%-22s = %5.2f : success %d/%d\n', name, lv, nOk, nRun);
        end
    end
    saveResultsTable(sens, fullfile(root, 'results', 'tables', 'sensitivity_results.csv'));
end
info = struct('date', datestr(now), 'version', version, 'configHash', configHash(cfg), ...
              'wallClock_s', toc(tStart));
save(fullfile(root, 'results', 'logs', 'benchmark.mat'), 'results', 'sens', 'cfg', 'variants', 'info');
fprintf('\nBenchmark finished in %.1f s. Tables in results/tables.\n', info.wallClock_s);
