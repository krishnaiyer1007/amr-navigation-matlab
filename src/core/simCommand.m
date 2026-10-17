function [ctx, cmd] = simCommand(ctx)
%SIMCOMMAND Tracking controller + controller-specific local safety layer.
%   Returns the REQUESTED body command [v w]; speed/acceleration limits are
%   applied afterwards by limitCommand (Safety Supervisor in Simulink).
    cfg = ctx.cfg; robot = ctx.robot;
    if isempty(ctx.path)
        nominal = [0 0];
        ctx.trackInfo = struct('closestIdx', 1, 'crossTrack', NaN);
    else
        [nominal, info] = computeTrackingCommand(robot.poseEst, ctx.path, ctx.ci, cfg, ctx.goal);
        ctx.ci = info.closestIdx;
        ctx.trackInfo = info;
    end
    mode = 0;
    state = struct('pose', robot.poseEst, 'v', robot.cmd(1), 'w', robot.cmd(2));
    switch ctx.controller
        case 'B0'
            cmd = nominal;
        case 'B1'
            [cmd, stopped] = proximityStop(nominal, state, ctx.pts, ctx.tracks, cfg);
            if stopped, mode = 2; end
        case 'P'
            if isempty(ctx.path)
                cmd = [0 0]; mode = 2;
            else
                keep = ~ctx.isMovPt & (ctx.inCl | ~ctx.isNew);
                staticPts = [ctx.pts(keep, :); ctx.obsPts; ctx.knownPts];
                dynObs = movingTrackMatrix(ctx.tracks);
                [cmd, dinfo] = selectSafeVelocity(state, nominal, ctx.path, ctx.ci, ...
                                                  staticPts, dynObs, cfg);
                mode = dinfo.mode;
            end
        otherwise
            error('simCommand:controller', 'Unknown controller %s', ctx.controller);
    end
    ctx.mode = mode;
end
