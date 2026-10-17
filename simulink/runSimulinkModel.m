function [result, log, simOut] = runSimulinkModel(scenarioId, controller, seed, cfg, rebuild)
%RUNSIMULINKMODEL Run simulink/amrNavigation.slx for one scenario.
%   [result, log] = runSimulinkModel('S05', 'P')
%   [result, log] = runSimulinkModel('S09', 'P', 7, cfg, true)   % rebuild model
%   Uses the same scenario definitions, seeds and metrics as runSimulation.
    if nargin < 2, controller = 'P'; end
    if nargin < 3, seed = 0; end
    if nargin < 4 || isempty(cfg), cfg = defaultConfig(); end
    if nargin < 5, rebuild = false; end
    here = fileparts(mfilename('fullpath'));
    mdlPath = fullfile(here, 'amrNavigation.slx');
    sc = scenarioDefinitions(scenarioId, seed);
    cfgRun = applyOverrides(cfg, sc.overrides);
    if rebuild || ~exist(mdlPath, 'file') || ~isequal(sc.overrides, struct())
        buildSimulinkModel(cfgRun, mdlPath);      % constants are inlined at build time
    end
    mdl = 'amrNavigation';
    load_system(mdlPath);
    % the model's InitFcn reads these, exactly as when Run is pressed
    vars = {'amrScenario', sc.id; 'amrController', controller; 'amrSeed', seed; ...
            'amrCfg', cfg; 'amrShowResults', false};
    for i = 1:size(vars, 1), assignin('base', vars{i,1}, vars{i,2}); end
    cleaner = onCleanup(@() evalin('base', 'clear amrCfg amrShowResults amrScenario amrController amrSeed')); %#ok<NASGU>
    simOut = sim(mdl);
    result = evalin('base', 'amrResult');
    log = evalin('base', 'amrLog');
    if ~isequal(sc.overrides, struct())
        buildSimulinkModel(cfg, mdlPath);          % restore default-constant model
    end
end
