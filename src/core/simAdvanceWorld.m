function ctx = simAdvanceWorld(ctx, t)
%SIMADVANCEWORLD Move scripted obstacles and activate hidden blockers (ground truth).
    ctx.world = advanceDynamicObstacles(ctx.world, t, ctx.cfg.dt);
end
