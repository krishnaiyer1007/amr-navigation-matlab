function ctx = simReplan(ctx, t)
%SIMREPLAN Blockage detection and debounced global replanning (controller P).
%   Triggers: path blocked for blockDebounce consecutive steps, periodic
%   replan when the observed map changed, large deviation, or no path.
%   A new path is accepted only if A* succeeds; otherwise the previous
%   path is kept unless it is blocked, in which case the robot stops.
    ctx.blockedNow = false;
    if ~ctx.useReplan, return; end
    cfg = ctx.cfg;
    if ~isempty(ctx.path)
        ctx.blockedNow = isPathBlocked(ctx.path, ctx.ci, ctx.obsPts, ctx.Rchk, cfg.planner.checkAhead);
    end
    if ctx.blockedNow
        ctx.blockCount = ctx.blockCount + 1;
        if isnan(ctx.blockedSince), ctx.blockedSince = t; end
    else
        ctx.blockCount = 0;
    end
    if isempty(ctx.path)
        ctNow = 0;
    else
        [~, ctNow] = closestPathIndex(ctx.path, ctx.robot.poseEst(1:2), ctx.ci);
    end
    due = (t - ctx.lastPlanT >= cfg.planner.replanPeriod && ctx.lm.version ~= ctx.planVersion);
    trig = ctx.blockCount >= cfg.planner.blockDebounce || due || ...
           ctNow > cfg.planner.deviationReplan || isempty(ctx.path);
    if ~(trig && (t - ctx.lastPlanT >= cfg.planner.minReplanInterval)), return; end

    tp = tic;
    [cand, newPath] = planGlobalPath(ctx.world.known | ctx.obsOcc, ctx.robot.poseEst(1:2), ctx.goal, cfg);
    ctx.planTimes(end+1) = toc(tp);
    ctx.lastPlanT = t; ctx.planVersion = ctx.lm.version;
    if cand.success
        ctx.path = newPath; ctx.ci = 1; ctx.blockCount = 0;
        ctx.result.replans = ctx.result.replans + 1;
        if ~isnan(ctx.blockedSince)
            ctx.latencies(end+1) = t - ctx.blockedSince;
        end
        ctx.blockedSince = NaN; ctx.noPathSince = NaN;
        ctx.log.paths{end+1} = struct('t', t, 'path', newPath, 'status', cand.status);
        ctx.log.events(end+1) = simEvent(t, 'replan', cand.status, ctx.robot.pose);
    else
        ctx.log.events(end+1) = simEvent(t, 'replan_fail', cand.status, ctx.robot.pose);
        if ctx.blockedNow || isempty(ctx.path)
            if ~isempty(ctx.path)
                ctx.log.paths{end+1} = struct('t', t, 'path', zeros(0,2), 'status', cand.status);
            end
            ctx.path = zeros(0, 2);
            if isnan(ctx.noPathSince), ctx.noPathSince = t; end
        end
    end
end
