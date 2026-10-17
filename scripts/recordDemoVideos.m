%RECORDDEMOVIDEOS Record the demonstration videos (README deliverable 18).
%   results/videos/<scenario>_<controller>.mp4, 20 fps, played at 4x real time.
%   Shows the known map with inflation, ground-truth blockers once they
%   appear, moving obstacles, the LiDAR returns, the active plan and the
%   robot footprint, with time, speed, clearance and controller mode.
root = setupPaths();
outDir = fullfile(root, 'results', 'videos');
if ~exist(outDir, 'dir'), mkdir(outDir); end
cfg = defaultConfig();
cfg.log.scans = true;
runs = {'S06', 'P',  'Sudden blockage: detection and replanning'; ...
        'S05', 'P',  'Two crossing obstacles: prediction and avoidance'; ...
        'S05', 'B0', 'Baseline without avoidance: collision'; ...
        'S11', 'P',  'Occluded obstacle emerging from an aisle'; ...
        'S07', 'P',  'No feasible route: safe stop'};
for i = 1:size(runs, 1)
    sc = scenarioDefinitions(runs{i,1});
    [r, lg] = runSimulation(sc, runs{i,2}, cfg);
    file = fullfile(outDir, sprintf('%s_%s.mp4', runs{i,1}, runs{i,2}));
    fprintf('%s %s (%s): %s -> %s\n', runs{i,1}, runs{i,2}, runs{i,3}, r.reason, file);
    animateScenario(sc, lg, cfg, struct('speed', 4, 'video', file, 'outcome', r.reason));
    close all;
end
