% testDeterminism  Identical seeds give identical trajectories and metrics.
cfg = defaultConfig();

%% Same seed, same result (noisy sensors)
sc = scenarioDefinitions('S09', 7);
[r1, l1] = runSimulation(sc, 'P', cfg);
[r2, l2] = runSimulation(sc, 'P', cfg);
assert(isequal(l1.pose, l2.pose), 'Trajectories differ');
assert(isequal(l1.cmd, l2.cmd), 'Commands differ');
f = {'success', 'reason', 'time', 'pathLength', 'minClearance', 'replans'};
for i = 1:numel(f)
    assert(isequaln(r1.(f{i}), r2.(f{i})), 'Metric %s differs', f{i});
end

%% Scenario randomisation is repeatable and seed dependent
a = scenarioDefinitions('S05', 3); b = scenarioDefinitions('S05', 3); c = scenarioDefinitions('S05', 4);
assert(isequal(a, b), 'Same seed gave different scenarios');
assert(~isequal([a.dyn.speed], [c.dyn.speed]), 'Different seeds gave identical scenarios');

%% Caller random state is not disturbed by scenario generation
rng(123); x1 = rand; rng(123); scenarioDefinitions('S04', 9); x2 = rand;
assert(x1 == x2, 'scenarioDefinitions changed the global RNG state');
