%EXPORTLINKEDINFRAMES High-resolution stills of key moments for sharing.
root = setupPaths();
out = fullfile(root, 'results', 'linkedin');
if ~exist(out, 'dir'), mkdir(out); end
cfg = defaultConfig(); cfg.log.scans = true;
sc = scenarioDefinitions('S05');
[r0, l0] = runSimulation(sc, 'B0', cfg);
[rp, lp] = runSimulation(sc, 'P', cfg);
tc = l0.t(end);
z = struct('xlim', [3.5 11], 'ylim', [4.2 10]);
z.title = sprintf('A* path only: collision at t = %.1f s', tc); z.file = fullfile(out, 'frame_S05_B0.png');
snapshotScenario(sc, l0, cfg, tc, z);
z.title = sprintf('Full system at the same moment: waits, clearance %.2f m', interp1(lp.t, lp.clearance, tc, 'nearest'));
z.file = fullfile(out, 'frame_S05_P.png');
snapshotScenario(sc, lp, cfg, tc, z);
z.title = 'Full system a few seconds later: passes behind the obstacle'; z.file = fullfile(out, 'frame_S05_P_later.png');
snapshotScenario(sc, lp, cfg, tc + 3.5, z);
sc = scenarioDefinitions('S06');
[r6, l6] = runSimulation(sc, 'P', cfg);
snapshotScenario(sc, l6, cfg, 15.5, struct('title', 'Sudden blockage: new obstacle seen, route replanned through the next aisle', ...
    'file', fullfile(out, 'frame_S06_P.png')));
sc = scenarioDefinitions('S11');
[r11, l11] = runSimulation(sc, 'P', cfg);
snapshotScenario(sc, l11, cfg, 9.5, struct('xlim', [4 16], 'ylim', [2 13], ...
    'title', 'Obstacle appears from behind a shelf: tracked, predicted, avoided', 'file', fullfile(out, 'frame_S11_P.png')));
close all;
fprintf('LinkedIn frames written to %s\n', out);
