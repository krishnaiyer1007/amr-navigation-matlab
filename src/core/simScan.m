function ctx = simScan(ctx)
%SIMSCAN Simulated LiDAR from the TRUE pose against the ground-truth world.
    ctx.scan = simulateRangeScan(ctx.robot.pose, ctx.world, ctx.cfg);
end
