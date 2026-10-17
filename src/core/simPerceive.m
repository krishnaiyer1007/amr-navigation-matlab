function ctx = simPerceive(ctx, t)
%SIMPERCEIVE Interpret the scan with the ESTIMATED pose: clustering,
%   tracking (moving obstacles) and the local observed-obstacle map.
    cfg = ctx.cfg;
    [pts, beamIdx] = scanToPoints(ctx.scan, ctx.robot.poseEst);
    isNew = classifyKnown(pts, ctx.lm);
    [ctx.tracks, isMovPt, ~, inCl] = estimateObstacleMotion(ctx.tracks, pts, beamIdx, isNew, ...
                                                       ctx.robot.poseEst(1:2), t, cfg);
    ctx.lm = updateLocalMap(ctx.lm, ctx.scan, ctx.robot.poseEst, pts, beamIdx, inCl, isMovPt, t, cfg);
    [ctx.obsOcc, ctx.obsPts] = observedGrid(ctx.lm);
    ctx.pts = pts; ctx.isMovPt = isMovPt; ctx.isNew = isNew; ctx.inCl = inCl;
end
