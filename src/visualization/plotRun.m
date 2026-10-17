function figs = plotRun(sc, res, log, cfg, outDir, tag)
%PLOTRUN Standard per-run figures (README section 14, figures 1-6).
%   Saves PNGs into outDir with prefix tag (e.g. 'S05_P'). Returns handles.
%   1 map + inflation + initial A* route    2 trajectory + time-labelled obstacles
%   3 heading / v / w vs time               4 clearance vs time with threshold
%   5 cross-track error vs time             6 replanning events on trajectory
    if nargin < 5, outDir = ''; end
    if nargin < 6, tag = sprintf('%s_%s', sc.id, res.controller); end
    cfg = applyOverrides(cfg, sc.overrides);
    world = initWorld(cfg, sc);
    lbl = sprintf('%s (%s) | controller %s | seed %d | %s', sc.id, sc.name, ...
                  res.controller, res.seed, res.reason);
    figs = gobjects(0);

    % --- Figure 1: occupancy grid, inflation, planned A* route
    f = figure('Name', [tag ' map'], 'Color', 'w', 'Position', [50 50 900 680]);
    ax = axes(f); drawMap(ax, world, struct('cfg', cfg, 'title', ['Occupancy grid, inflation and A* route - ' lbl]));
    p0 = log.paths{1}.path;
    h = gobjects(0); names = {};
    if ~isempty(p0), h(end+1) = plot(ax, p0(:,1), p0(:,2), 'b-', 'LineWidth', 2); names{end+1} = 'Initial A* path (smoothed)'; end
    h(end+1) = plot(ax, sc.start(1), sc.start(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 9); names{end+1} = 'Start';
    h(end+1) = plot(ax, sc.goal(1), sc.goal(2), 'rp', 'MarkerFaceColor', 'r', 'MarkerSize', 13); names{end+1} = 'Goal';
    legend(ax, h, names, 'Location', 'southoutside', 'Orientation', 'horizontal');
    figs(end+1) = save1(f, outDir, [tag '_fig01_map_astar']);

    % --- Figure 2: trajectory with time-labelled moving obstacles
    f = figure('Name', [tag ' trajectory'], 'Color', 'w', 'Position', [60 60 900 680]);
    ax = axes(f); drawMap(ax, world, struct('cfg', cfg, 'title', ['Robot trajectory and moving obstacles - ' lbl]));
    h = gobjects(0); names = {};
    if ~isempty(p0), h(end+1) = plot(ax, p0(:,1), p0(:,2), 'b--', 'LineWidth', 1); names{end+1} = 'Initial plan'; end
    if ~isempty(log.t)
        h(end+1) = plot(ax, log.pose(:,1), log.pose(:,2), 'k-', 'LineWidth', 2); names{end+1} = 'Robot trajectory';
        nd = size(log.dynPos, 3);
        cols = lines(max(nd, 1));
        stepIdx = 1:max(1, round(2 / cfg.dt)):numel(log.t);
        for i = 1:nd
            P = log.dynPos(:,:,i);
            hh = plot(ax, P(:,1), P(:,2), '-', 'Color', [cols(i,:) 0.6], 'LineWidth', 1);
            if i == 1, h(end+1) = hh; names{end+1} = 'Moving obstacle paths'; end %#ok<AGROW>
            for k = stepIdx
                th = linspace(0, 2*pi, 24);
                r = world.dyn(i).r;
                plot(ax, P(k,1) + r*cos(th), P(k,2) + r*sin(th), '-', 'Color', cols(i,:));
                text(ax, P(k,1), P(k,2), sprintf('%.0f', log.t(k)), 'FontSize', 7, ...
                     'HorizontalAlignment', 'center', 'Color', cols(i,:));
            end
        end
        for k = stepIdx
            text(ax, log.pose(k,1), log.pose(k,2) + 0.35, sprintf('%.0f', log.t(k)), 'FontSize', 7, ...
                 'HorizontalAlignment', 'center');
        end
    end
    h(end+1) = plot(ax, sc.start(1), sc.start(2), 'go', 'MarkerFaceColor', 'g'); names{end+1} = 'Start';
    h(end+1) = plot(ax, sc.goal(1), sc.goal(2), 'rp', 'MarkerFaceColor', 'r', 'MarkerSize', 12); names{end+1} = 'Goal';
    legend(ax, h, names, 'Location', 'southoutside', 'Orientation', 'horizontal');
    figs(end+1) = save1(f, outDir, [tag '_fig02_trajectory']);

    if isempty(log.t), return; end

    % --- Figure 3: heading, v, w vs time
    f = figure('Name', [tag ' states'], 'Color', 'w', 'Position', [70 70 900 700]);
    subplot(3,1,1); plot(log.t, log.pose(:,3) * 180/pi, 'k'); grid on;
    ylabel('Heading (deg)'); title(['Robot heading, linear and angular speed - ' lbl], 'Interpreter', 'none');
    subplot(3,1,2); plot(log.t, log.vel(:,1), 'b', log.t, log.cmd(:,1), 'r--'); grid on;
    ylabel('v (m/s)'); legend('Actual', 'Commanded', 'Location', 'best');
    subplot(3,1,3); plot(log.t, log.vel(:,2), 'b', log.t, log.cmd(:,2), 'r--'); grid on;
    ylabel('\omega (rad/s)'); xlabel('Time (s)'); legend('Actual', 'Commanded', 'Location', 'best');
    figs(end+1) = save1(f, outDir, [tag '_fig03_states']);

    % --- Figure 4: clearance vs time
    f = figure('Name', [tag ' clearance'], 'Color', 'w', 'Position', [80 80 900 400]);
    plot(log.t, log.clearance, 'k', 'LineWidth', 1.2); hold on;
    plot(log.t([1 end]), cfg.safety.margin * [1 1], 'r--');
    plot(log.t([1 end]), [0 0], 'r-', 'LineWidth', 1.5);
    grid on; xlabel('Time (s)'); ylabel('Clearance (m)');
    legend('Min. distance robot surface to obstacle (static capped at 1 m)', ...
           sprintf('Safety margin %.2f m', cfg.safety.margin), 'Collision', 'Location', 'best');
    title(['Distance to nearest obstacle - ' lbl], 'Interpreter', 'none');
    figs(end+1) = save1(f, outDir, [tag '_fig04_clearance']);

    % --- Figure 5: tracking error vs time
    f = figure('Name', [tag ' tracking'], 'Color', 'w', 'Position', [90 90 900 400]);
    plot(log.t, log.crossTrack, 'b'); grid on; hold on;
    plot(log.t([1 end]), 0.2 * [1 1], 'r--');
    xlabel('Time (s)'); ylabel('Cross-track error (m)');
    legend('Distance to active reference path', '0.20 m target (static maps)', 'Location', 'best');
    title(['Reference-path tracking error - ' lbl], 'Interpreter', 'none');
    figs(end+1) = save1(f, outDir, [tag '_fig05_tracking']);

    % --- Figure 6: replanning events
    f = figure('Name', [tag ' replans'], 'Color', 'w', 'Position', [100 100 900 680]);
    ax = axes(f); drawMap(ax, world, struct('cfg', cfg, 'title', ['Planning and replanning events - ' lbl]));
    h = gobjects(0); names = {};
    np = numel(log.paths);
    cmap = parula(max(np, 2));
    for i = 1:np
        P = log.paths{i}.path;
        if isempty(P), continue; end
        hh = plot(ax, P(:,1), P(:,2), '-', 'Color', cmap(i,:), 'LineWidth', 1);
        if i == 1, h(end+1) = hh; names{end+1} = 'Plans (colour = time order)'; end %#ok<AGROW>
    end
    h(end+1) = plot(ax, log.pose(:,1), log.pose(:,2), 'k-', 'LineWidth', 2); names{end+1} = 'Trajectory';
    E = log.events;
    rp = strcmp({E.type}, 'replan'); rf = strcmp({E.type}, 'replan_fail');
    if any(rp), h(end+1) = plot(ax, [E(rp).x], [E(rp).y], 'ms', 'MarkerFaceColor', 'm'); names{end+1} = 'Replan (valid path)'; end
    if any(rf), h(end+1) = plot(ax, [E(rf).x], [E(rf).y], 'rx', 'MarkerSize', 10, 'LineWidth', 2); names{end+1} = 'Replan failed'; end
    blk = find(log.blocked);
    if ~isempty(blk), h(end+1) = plot(ax, log.pose(blk,1), log.pose(blk,2), 'r.', 'MarkerSize', 8); names{end+1} = 'Path blocked detected'; end
    legend(ax, h, names, 'Location', 'southoutside', 'Orientation', 'horizontal');
    figs(end+1) = save1(f, outDir, [tag '_fig06_replans']);
end

function f = save1(f, outDir, name)
    if ~isempty(outDir)
        saveFigure(f, fullfile(outDir, [name '.png']));
    end
end
