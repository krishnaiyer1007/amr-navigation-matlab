function amrModelInit()
%AMRMODELINIT InitFcn of amrNavigation.slx: prepares a run when Run is pressed.
%   Reads optional base-workspace variables (defaults in brackets):
%     amrScenario   scenario id            ['S05']
%     amrController 'B0' | 'B1' | 'P'      ['P']
%     amrSeed       scenario / noise seed  [0]
%     amrCfg        configuration struct   [defaultConfig()]
%   Initialises the shared navigation context (amrSimBridge) and sets
%   amrStartPose for the Pose State unit delay. runSimulinkModel sets the
%   same variables, so the toolstrip Run button and the script path run
%   identical code.
    scId = baseVar('amrScenario', 'S05');
    ctrl = baseVar('amrController', 'P');
    seed = baseVar('amrSeed', 0);
    cfg = baseVar('amrCfg', []);
    if isempty(cfg), cfg = defaultConfig(); end
    sc = scenarioDefinitions(scId, seed);

    % physical constants are inlined in the model at build time: refuse a
    % scenario whose overrides change them (runSimulinkModel rebuilds instead)
    cfgRun = applyOverrides(cfg, sc.overrides);
    built = [];
    mws = get_param(bdroot, 'ModelWorkspace');
    if hasVariable(mws, 'amrBuiltConstants'), built = getVariable(mws, 'amrBuiltConstants'); end
    if isstruct(built) && isfield(built, 'robot') && ...
            (~isequal(built.robot, cfgRun.robot) || ~isequal(built.wheelPID, cfgRun.wheelPID) || built.dt ~= cfgRun.dt)
        error('amrNavigation:rebuild', ['Scenario %s changes robot or controller constants that are built ', ...
            'into the model. Run it with runSimulinkModel(''%s'', ''%s'', %d), which rebuilds the model.'], ...
            sc.id, sc.id, ctrl, seed);
    end
    [startPose, done] = amrSimBridge('init', sc, ctrl, cfg, struct('seed', seed));
    if done
        error('amrNavigation:noPlan', 'Scenario %s: no initial path exists, nothing to simulate.', sc.id);
    end
    assignin('base', 'amrStartPose', startPose);
    fprintf('amrNavigation: %s (%s), controller %s, seed %d\n', sc.id, sc.name, ctrl, seed);
end

function v = baseVar(name, default)
    if evalin('base', sprintf('exist(''%s'', ''var'')', name))
        v = evalin('base', name);
    else
        v = default;
    end
end
