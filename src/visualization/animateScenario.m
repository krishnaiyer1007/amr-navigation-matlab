function animateScenario(sc, log, cfg, opts)
%ANIMATESCENARIO Live animation of a logged run.
%   animateScenario(sc, log, cfg)            play in a figure window
%   opts.speed     playback speed-up (default 4)
%   opts.video     filename (.mp4 / .avi) to record with VideoWriter ('' = none)
%   opts.showScan  draw LiDAR points when log.scans exists (default true)
%   opts.outcome   result text shown on the final frame (e.g. result.reason)
%   Shows: known map + inflation, ground-truth blockers once active (red),
%   moving obstacles (ground truth), active plan, trajectory, footprint,
%   heading and LiDAR returns (perception, drawn separately from truth).
    if nargin < 4, opts = struct(); end
    if ~isfield(opts, 'speed'), opts.speed = 4; end
    if ~isfield(opts, 'video'), opts.video = ''; end
    if ~isfield(opts, 'showScan'), opts.showScan = true; end
    cfg = applyOverrides(cfg, sc.overrides);
    world = initWorld(cfg, sc);
    if isempty(log.t), warning('animateScenario:empty', 'Empty log'); return; end

    f = figure('Name', ['Animation ' sc.id], 'Color', 'w', 'Position', [100 100 1000 760]);
    if ~isempty(opts.video), set(f, 'Visible', 'off'); end
    ax = axes(f);
    drawMap(ax, world, struct('cfg', cfg, 'blockers', false, ...
            'title', sprintf('%s - %s', sc.id, sc.name)));
    nd = size(log.dynPos, 3);
    th = linspace(0, 2*pi, 30);
    hPath = plot(ax, NaN, NaN, 'b-', 'LineWidth', 1.5);
    hTraj = plot(ax, NaN, NaN, 'k-', 'LineWidth', 1.5);
    hRob = patch(ax, NaN, NaN, [0.2 0.5 1], 'FaceAlpha', 0.6);
    hHead = plot(ax, NaN, NaN, 'k-', 'LineWidth', 2);
    hScan = plot(ax, NaN, NaN, '.', 'Color', [1 0.5 0], 'MarkerSize', 6);
    hDyn = gobjects(nd, 1);
    for i = 1:nd, hDyn(i) = patch(ax, NaN, NaN, [0.9 0.3 0.3], 'FaceAlpha', 0.7); end
    hBlk = gobjects(numel(world.blockers), 1);
    for i = 1:numel(world.blockers)
        r = world.blockers(i).rect;
        hBlk(i) = patch(ax, r([1 3 3 1]), r([2 2 4 4]), [0.85 0.2 0.2], 'Visible', 'off');
    end
    plot(ax, sc.goal(1), sc.goal(2), 'rp', 'MarkerFaceColor', 'r', 'MarkerSize', 13);
    hTxt = text(ax, 0.3, world.height - 0.5, '', 'FontWeight', 'bold', 'BackgroundColor', 'w');

    vw = [];
    if ~isempty(opts.video)
        try
            vw = VideoWriter(opts.video, 'MPEG-4');
        catch
            vw = VideoWriter(opts.video);
        end
        vw.FrameRate = 20; open(vw);
        try, theme(f, 'light'); catch, end
    end
    pathTimes = cellfun(@(p) p.t, log.paths);
    step = max(1, round(opts.speed * (1/20) / cfg.dt));
    ang = linspace(-cfg.sensor.fov/2, cfg.sensor.fov/2, cfg.sensor.numBeams) * pi/180;
    R = cfg.robot.radius;
    frames = unique([1:step:numel(log.t), numel(log.t)]);     % always end on the final step
    outcome = '';
    if isfield(opts, 'outcome'), outcome = opts.outcome; end
    for k = frames
        p = log.pose(k,:); t = log.t(k);
        set(hRob, 'XData', p(1) + R*cos(th), 'YData', p(2) + R*sin(th));
        set(hHead, 'XData', p(1) + [0 R*cos(p(3))], 'YData', p(2) + [0 R*sin(p(3))]);
        set(hTraj, 'XData', log.pose(1:k,1), 'YData', log.pose(1:k,2));
        ip = find(pathTimes <= t, 1, 'last');
        P = log.paths{ip}.path;
        if isempty(P), set(hPath, 'XData', NaN, 'YData', NaN);
        else, set(hPath, 'XData', P(:,1), 'YData', P(:,2)); end
        for i = 1:nd
            c = log.dynPos(k,:,i);
            set(hDyn(i), 'XData', c(1) + world.dyn(i).r*cos(th), 'YData', c(2) + world.dyn(i).r*sin(th));
        end
        for i = 1:numel(world.blockers)
            if t >= world.blockers(i).tAppear, set(hBlk(i), 'Visible', 'on'); end
        end
        if opts.showScan && isfield(log, 'scans') && ~isempty(log.scans{k})
            rr = log.scans{k}; pe = log.poseEst(k,:);
            v = ~isnan(rr);
            set(hScan, 'XData', pe(1) + rr(v).*cos(pe(3)+ang(v)), 'YData', pe(2) + rr(v).*sin(pe(3)+ang(v)));
        end
        modes = {'normal', 'EVASIVE', 'STOP'};
        msg = sprintf('t = %5.1f s   v = %.2f m/s   clearance = %.2f m   mode = %s', ...
            t, log.vel(k,1), log.clearance(k), modes{log.mode(k) + 1});
        if k == frames(end) && ~isempty(outcome)
            msg = sprintf('%s   RESULT: %s', msg, upper(outcome));
        end
        set(hTxt, 'String', msg);
        drawnow;
        if ~isempty(vw)
            % print works in -batch / headless sessions where getframe may not
            img = print(f, '-RGBImage', '-r100');
            nRep = 1;
            if k == frames(end), nRep = 40; end       % hold the final frame for 2 s
            for rep = 1:nRep, writeVideo(vw, img); end
        else
            pause(0.01);
        end
        if ~ishandle(f), break; end
    end
    if ~isempty(vw), close(vw); fprintf('Video saved to %s\n', opts.video); end
end
