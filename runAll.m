%RUNALL Regenerate every result of the project from scratch.
%   1) unit + integration tests      4) benchmark (90 paired runs + sensitivity)
%   2) baseline controllers B0, B1   5) figures
%   3) full system P (+ figures)     6) Simulink build + MATLAB/Simulink comparison
%   Run from the project root:  runAll
addpath(fullfile(fileparts(mfilename('fullpath')), 'src', 'utils'));
root = setupPaths();
cd(root);
generateMaps;
testResults = runAllTests();
runScenarios;
runBaseline;
makeFigures = true; runFullSystem;
quick = false; doSensitivity = true; runBenchmark;
plotResults;
if exist('sim', 'file') == 2 || exist('sim', 'builtin') == 5
    compareSimulink;
else
    fprintf('Simulink not installed: skipping model build and comparison.\n');
end
fprintf('\nAll done. See results/tables, results/figures and results/logs.\n');
