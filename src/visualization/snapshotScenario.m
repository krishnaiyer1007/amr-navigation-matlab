function f = snapshotScenario(sc, log, cfg, t, opts)
%SNAPSHOTSCENARIO Publication-quality still of a logged run at time t.
%   f = snapshotScenario(sc, log, cfg, t, opts)
%   opts.xlim, opts.ylim  zoom window (m)      opts.title  title text
%   opts.file             PNG to write (exportgraphics, 200 dpi)
%   Draws the known map with inflation, active blockers, moving obstacles
%   with their recent trail, the active plan, the trajectory so far, the
%   LiDAR returns (when logged) and the robot footprint with heading.
    if nargin < 5, opts = struct(); end
    cfg = applyOverrides(cfg, sc.overrides);
    world = initWorld(cfg, sc);
    [~, k] = min(abs(log.t - t));
    f = figure('Color', 'w', 'Position', [100 100 1200 900], 'Visible', 'off');
    try, theme(f, 'light'); catch, end
    ax = axes(f);
    drawMap(ax, world, struct('cfg', cfg, 'blockers', false));
    for i = 1:numel(world.blockers)
        if log.t(k) >= world.blockers(i).tAppear
            r = world.blockers(i).rect;
            patch(ax, r([1 3 3 1]), r([2 2 4 4]), [0.85 0.2 0.2], 'EdgeColor', [0.6 0 0]);
        end
    end
    pathTimes = cellfun(@(p) p.t, log.paths);
    P = log.paths{find(pathTimes <= log.t(k), 1, 'last')}.path;
    if ~isempty(P), plot(ax, P(:,1), P(:,2), '-', 'Color', [0.15 0.35 0.9], 'LineWidth', 2.5); end
    plot(ax, log.pose(1:k,1), log.pose(1:k,2), 'k-', 'LineWidth', 2.5);
    th = linspace(0, 2*pi, 60);
    trail = max(1, k - round(3 / cfg.dt)):k;
    for i = 1:size(log.dynPos, 3)
        plot(ax, log.dynPos(trail,1,i), log.dynPos(trail,2,i), ':', 'Color', [0.85 0.3 0.3], 'LineWidth', 2);
        c = log.dynPos(k,:,i); r = world.dyn(i).r;
        patch(ax, c(1) + r*cos(th), c(2) + r*sin(th), [0.9 0.35 0.35], 'EdgeColor', [0.55 0.1 0.1], 'LineWidth', 1.5);
    end
    if isfield(log, 'scans') && ~isempty(log.scans{k})
        ang = linspace(-cfg.sensor.fov/2, cfg.sensor.fov/2, cfg.sensor.numBeams) * pi/180;
        rr = log.scans{k}; pe = log.poseEst(k,:); v = ~isnan(rr);
        plot(ax, pe(1) + rr(v).*cos(pe(3)+ang(v)), pe(2) + rr(v).*sin(pe(3)+ang(v)), '.', ...
             'Color', [1 0.55 0], 'MarkerSize', 10);
    end
    p = log.pose(k,:); R = cfg.robot.radius;
    patch(ax, p(1) + R*cos(th), p(2) + R*sin(th), [0.3 0.55 1], 'EdgeColor', [0 0.2 0.6], 'LineWidth', 1.5);
    plot(ax, p(1) + [0 R*cos(p(3))], p(2) + [0 R*sin(p(3))], 'k-', 'LineWidth', 2.5);
    plot(ax, sc.goal(1), sc.goal(2), 'p', 'MarkerSize', 22, 'MarkerFaceColor', [0.9 0.1 0.1], 'MarkerEdgeColor', 'k');
    if isfield(opts, 'xlim'), xlim(ax, opts.xlim); end
    if isfield(opts, 'ylim'), ylim(ax, opts.ylim); end
    ax.FontSize = 14;
    if isfield(opts, 'title'), title(ax, opts.title, 'FontSize', 18, 'Interpreter', 'none'); end
    if isfield(opts, 'file'), exportgraphics(f, opts.file, 'Resolution', 200); end
end
