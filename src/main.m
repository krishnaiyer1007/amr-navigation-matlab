%MAIN Demonstration entry point: one scenario, live animation, figures.
%   Change scenarioId / controller below. Options: 'B0', 'B1', 'P'.
%   Run from the project root:  run('src/main.m')
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'utils')); setupPaths();
cfg = defaultConfig();
cfg.log.scans = true;               % needed to draw LiDAR returns in the animation

scenarioId = 'S05';                 % two crossing obstacles
controller = 'P';
seed = 0;                           % 0 = nominal scenario, >0 = seeded variant

sc = scenarioDefinitions(scenarioId, seed);
fprintf('Running %s (%s) with controller %s ...\n', sc.id, sc.name, controller);
[result, log] = runSimulation(sc, controller, cfg, struct('verbose', true));
disp(result);

animateScenario(sc, log, cfg, struct('speed', 3, 'outcome', result.reason));
plotRun(sc, result, log, cfg);
