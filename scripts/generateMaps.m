%GENERATEMAPS Save the warehouse map and scenario definitions to maps/warehouse01.mat
root = setupPaths();
cfg = defaultConfig();
warehouse = createMap(cfg, 'warehouse');          %#ok<NASGU>
ids = scenarioDefinitions('list');
scenarios = cellfun(@(id) scenarioDefinitions(id), ids);   %#ok<NASGU>
save(fullfile(root, 'maps', 'warehouse01.mat'), 'warehouse', 'scenarios', 'cfg');
fprintf('Saved maps/warehouse01.mat (%d scenarios)\n', numel(ids));
