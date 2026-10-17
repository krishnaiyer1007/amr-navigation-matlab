%TESTRUNBUTTON Check that amrNavigation.slx runs from its Run button path.
%   Simulates the model with sim() and NO preparation other than the
%   optional base variables, exactly what the toolstrip Run button does,
%   and compares the outcome with runSimulation.
root = setupPaths();
cfg = defaultConfig();
buildSimulinkModel(cfg);
load_system(fullfile(root, 'simulink', 'amrNavigation.slx'));
evalin('base', 'clear amrScenario amrController amrSeed amrCfg');
assignin('base', 'amrShowResults', false);
sim('amrNavigation');                                   % default: S05, P, seed 0
rb = evalin('base', 'amrResult'); lb = evalin('base', 'amrLog');
[rm, lm] = runSimulation(scenarioDefinitions('S05'), 'P', cfg);
assert(strcmp(rb.reason, rm.reason) && isequal(lb.pose, lm.pose), 'Run button result differs');
fprintf('Run button (default S05/P): %s, %.1f s, identical to runSimulation\n', rb.reason, rb.time);
assignin('base', 'amrScenario', 'S06');
sim('amrNavigation');
rb = evalin('base', 'amrResult');
fprintf('Run button with amrScenario = ''S06'': %s, %.1f s, replans %d\n', rb.reason, rb.time, rb.replans);
% a scenario that changes robot constants must be refused with a clear message
assignin('base', 'amrScenario', 'S10');
try
    sim('amrNavigation'); error('S10 should have been refused');
catch err
    assert(contains(err.message, 'runSimulinkModel') || contains(getReport(err), 'runSimulinkModel'), err.message);
    fprintf('S10 refused as expected (robot constants differ): message points to runSimulinkModel\n');
end
evalin('base', 'clear amrScenario amrShowResults');
r10 = runSimulinkModel('S10', 'P');
fprintf('runSimulinkModel(''S10''): %s, %.1f s (model rebuilt with vMax 1.0, then restored)\n', r10.reason, r10.time);
close_system('amrNavigation', 0);
