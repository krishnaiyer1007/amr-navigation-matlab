function ctx = simSense(ctx, t)
%SIMSENSE Advance the world, simulate the LiDAR and update perception.
%   Equivalent to simAdvanceWorld -> simScan -> simPerceive (the Simulink
%   model calls the three stages from separate subsystems).
    ctx = simAdvanceWorld(ctx, t);
    ctx = simScan(ctx);
    ctx = simPerceive(ctx, t);
end
