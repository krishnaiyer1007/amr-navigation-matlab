function ctx = simPostStep(ctx, k, t, cmd, stepTime)
%SIMPOSTSTEP Ground-truth collision check, logging and termination tests.
%   Must be called after the plant update (ctx.robot holds the new state).
    cfg = ctx.cfg; robot = ctx.robot; dt = cfg.dt;
    [collided, clr] = checkFootprintCollision(robot.pose, ctx.world, cfg);
    L = ctx.log;
    L.t(k) = t + dt; L.pose(k,:) = robot.pose; L.poseEst(k,:) = robot.poseEst;
    L.cmd(k,:) = cmd; L.vel(k,:) = [robot.v robot.w]; L.wheel(k,:) = robot.wheel;
    L.clearance(k) = clr; L.crossTrack(k) = ctx.trackInfo.crossTrack; L.mode(k) = ctx.mode;
    L.nTracks(k) = numel(ctx.tracks.list); L.nMoving(k) = size(movingTrackMatrix(ctx.tracks), 1);
    L.blocked(k) = ctx.blockedNow;
    for i = 1:numel(ctx.world.dyn), L.dynPos(k,:,i) = ctx.world.dyn(i).pos; end
    if ctx.opts.logScans, L.scans{k} = ctx.scan.ranges; end
    L.stepTime(k) = stepTime;
    ctx.log = L; ctx.k = k;

    if collided
        ctx.result.collisions = 1; ctx.result.reason = 'collision';
        ctx.log.events(end+1) = simEvent(t + dt, 'collision', '', robot.pose);
        ctx.done = true; return;
    end
    if norm(robot.pose(1:2) - ctx.goal) <= cfg.goal.tolerance
        ctx.result.success = true; ctx.result.reason = 'success';
        ctx.log.events(end+1) = simEvent(t + dt, 'goal', '', robot.pose);
        ctx.done = true; return;
    end
    if ~isnan(ctx.noPathSince) && t - ctx.noPathSince >= cfg.fail.unreachableHold && abs(robot.v) < 0.02
        ctx.result.reason = 'unreachable';
        ctx.log.events(end+1) = simEvent(t + dt, 'unreachable', 'safe stop', robot.pose);
        ctx.done = true; return;
    end
    if norm(robot.pose(1:2) - ctx.anchor) > 0.3
        ctx.anchor = robot.pose(1:2); ctx.anchorT = t;
    elseif t - ctx.anchorT > cfg.fail.stallTime
        ctx.result.reason = 'stall';
        ctx.log.events(end+1) = simEvent(t + dt, 'stall', '', robot.pose);
        ctx.done = true; return;
    end
    if k >= ctx.N, ctx.done = true; end      % timeout (reason stays 'timeout')
    if ctx.opts.verbose && mod(k, 200) == 0
        fprintf('  t=%6.1f  pos=(%5.2f,%5.2f)  clr=%5.2f  mode=%d\n', t, robot.pose(1), robot.pose(2), clr, ctx.mode);
    end
end
