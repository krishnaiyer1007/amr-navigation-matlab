%COMPARESIMULINK Verify the Simulink model reproduces the MATLAB implementation.
%   Same scenario, seed and configuration through runSimulation and
%   runSimulinkModel. Reports maximum pose / command differences, outcome
%   agreement and saves results/tables/simulink_comparison.csv plus an
%   overlay figure. Documented tolerance: 1e-6 m / 1e-6 rad.
root = setupPaths();
cfg = defaultConfig();
cases = {'S01', 'P', 0; 'S04', 'P', 0; 'S06', 'P', 0; 'S05', 'B1', 0; 'S12', 'P', 3};
tol = 1e-6;
rows = cell(size(cases, 1), 9);
buildSimulinkModel(cfg);
for i = 1:size(cases, 1)
    sc = scenarioDefinitions(cases{i,1}, cases{i,3});
    [rm, lm] = runSimulation(sc, cases{i,2}, cfg, struct('seed', cases{i,3}));
    t0 = tic;
    [rs, ls] = runSimulinkModel(cases{i,1}, cases{i,2}, cases{i,3}, cfg);
    tSl = toc(t0);
    n = min(size(lm.pose, 1), size(ls.pose, 1));
    dPos = max(sqrt(sum((lm.pose(1:n,1:2) - ls.pose(1:n,1:2)).^2, 2)));
    dYaw = max(abs(wrapAngle(lm.pose(1:n,3) - ls.pose(1:n,3))));
    dCmd = max(max(abs(lm.cmd(1:n,:) - ls.cmd(1:n,:))));
    same = strcmp(rm.reason, rs.reason) && size(lm.pose, 1) == size(ls.pose, 1);
    pass = same && dPos < tol && dYaw < tol;
    rows(i,:) = {cases{i,1}, cases{i,2}, cases{i,3}, rm.reason, rs.reason, dPos, dYaw, dCmd, double(pass)};
    fprintf('%s %-2s seed %d | MATLAB %-9s %5.1f s | Simulink %-9s %5.1f s | max dPos %.2e m, dYaw %.2e rad, dCmd %.2e | %s (Simulink wall %.1f s)\n', ...
        cases{i,1}, cases{i,2}, cases{i,3}, rm.reason, rm.time, rs.reason, rs.time, dPos, dYaw, dCmd, ...
        passStr(pass), tSl);
    if i == 3
        f = figure('Color', 'w', 'Position', [100 100 900 650]);
        w = initWorld(cfg, sc); ax = axes(f);
        drawMap(ax, w, struct('cfg', cfg, 'title', sprintf('MATLAB vs Simulink trajectory - %s (%s), controller %s', sc.id, sc.name, cases{i,2})));
        h1 = plot(ax, lm.pose(:,1), lm.pose(:,2), 'b-', 'LineWidth', 3);
        h2 = plot(ax, ls.pose(:,1), ls.pose(:,2), 'r--', 'LineWidth', 1.5);
        legend(ax, [h1 h2], {'MATLAB script', 'Simulink model'}, 'Location', 'southoutside');
        saveFigure(f, fullfile(root, 'results', 'figures', 'fig10b_matlab_vs_simulink.png'));
    end
end
% figure 10: Simulink architecture diagram
try
    load_system(fullfile(root, 'simulink', 'amrNavigation.slx'));
    print('-samrNavigation', '-dpng', '-r150', fullfile(root, 'results', 'figures', 'fig10_simulink_architecture.png'));
catch err
    warning('compareSimulink:print', 'Could not export the model diagram: %s', err.message);
end
writeCsv(fullfile(root, 'results', 'tables', 'simulink_comparison.csv'), ...
    {'scenario', 'controller', 'seed', 'matlabOutcome', 'simulinkOutcome', 'maxPosDiff_m', 'maxYawDiff_rad', 'maxCmdDiff', 'pass'}, rows);

function s = passStr(ok)
    if ok, s = 'PASS'; else, s = 'FAIL'; end
end
