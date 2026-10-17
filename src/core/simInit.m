function ctx = simInit(sc, controller, cfg, opts)
%SIMINIT Create the simulation/navigation context for one run.
%   Shared by runSimulation (MATLAB loop) and the Simulink model so both
%   execute identical navigation code. Stages per step:
%     simSense -> simReplan -> simCommand -> limitCommand -> plant -> simPostStep
    if nargin < 4, opts = struct(); end
    if ~isfield(opts, 'verbose'), opts.verbose = false; end
    if ~isfield(opts, 'seed'), opts.seed = sc.seed; end
    cfg = applyOverrides(cfg, sc.overrides);
    if ~isfield(opts, 'logScans'), opts.logScans = cfg.log.scans; end
    rng(1000 + opts.seed, 'twister');          % sensor / odometry noise stream

    ctx.sc = sc; ctx.controller = upper(controller); ctx.cfg = cfg; ctx.opts = opts;
    ctx.N = round(cfg.duration / cfg.dt);
    ctx.world = initWorld(cfg, sc);
    ctx.robot = initializeRobot(cfg, sc.start);
    ctx.lm = initializeLocalMap(ctx.world, cfg);
    ctx.tracks = initializeTracks();
    ctx.knownPts = knownBoundaryPoints(ctx.world.known, ctx.world.res);
    ctx.goal = sc.goal(:)';
    ctx.Rchk = cfg.robot.radius + 0.5 * cfg.safety.margin;
    ctx.useReplan = strcmp(ctx.controller, 'P');

    % initial global plan on the prior map
    tp = tic;
    [plan, path] = planGlobalPath(ctx.world.known, ctx.robot.poseEst(1:2), ctx.goal, cfg);
    ctx.planTimes = toc(tp);
    ctx.path = path;
    ctx.ci = 1; ctx.lastPlanT = 0; ctx.planVersion = ctx.lm.version; ctx.blockCount = 0;
    ctx.blockedSince = NaN; ctx.noPathSince = NaN; ctx.latencies = [];
    ctx.anchor = ctx.robot.pose(1:2); ctx.anchorT = 0;
    ctx.blockedNow = false; ctx.trackInfo = struct('closestIdx', 1, 'crossTrack', NaN);
    ctx.mode = 0; ctx.k = 0; ctx.done = false;
    ctx.pts = zeros(0, 2); ctx.isMovPt = false(0, 1); ctx.isNew = false(0, 1);
    ctx.inCl = false(0, 1); ctx.obsOcc = false(ctx.world.ny, ctx.world.nx); ctx.obsPts = zeros(0, 2);
    ctx.scan = [];

    N = ctx.N; nd = numel(ctx.world.dyn);
    log = struct();
    log.t = nan(N, 1); log.pose = nan(N, 3); log.poseEst = nan(N, 3);
    log.cmd = nan(N, 2); log.vel = nan(N, 2); log.wheel = nan(N, 2);
    log.clearance = nan(N, 1); log.crossTrack = nan(N, 1); log.mode = zeros(N, 1);
    log.nTracks = zeros(N, 1); log.nMoving = zeros(N, 1); log.blocked = false(N, 1);
    log.stepTime = nan(N, 1);
    log.dynPos = nan(N, 2, nd);
    log.paths = {struct('t', 0, 'path', path, 'status', plan.status)};
    log.events = struct('t', {}, 'type', {}, 'detail', {}, 'x', {}, 'y', {});
    log.events(end+1) = simEvent(0, 'plan', plan.status, ctx.robot.pose);
    if opts.logScans, log.scans = cell(N, 1); end
    ctx.log = log;

    ctx.result = struct('scenario', sc.id, 'controller', ctx.controller, 'seed', opts.seed, ...
        'success', false, 'reason', 'timeout', 'time', NaN, ...
        'pathLength', NaN, 'refLength', pathLen(path), 'efficiency', NaN, ...
        'meanTrackErr', NaN, 'minClearance', NaN, 'collisions', 0, ...
        'replans', 0, 'replanLatencyMean', NaN, 'replanLatencyMax', NaN, ...
        'stepTimeMean', NaN, 'stepTimeMax', NaN, 'planTimeMean', NaN, ...
        'controlEffort', NaN, 'stopTime', 0, 'evasiveTime', 0, 'finalDist', NaN);
    if ~plan.success
        ctx.result.reason = ['invalid_path:' plan.status];
        if strcmpi(sc.expect, 'unreachable'), ctx.result.reason = 'unreachable'; end
        ctx.done = true;
    end
end

function L = pathLen(p)
    if size(p, 1) < 2, L = NaN; else, L = sum(sqrt(sum(diff(p).^2, 2))); end
end
