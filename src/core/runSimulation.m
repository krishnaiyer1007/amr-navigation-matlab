function [result, log] = runSimulation(sc, controller, cfg, opts)
%RUNSIMULATION Closed-loop simulation of one scenario with one controller.
%   [result, log] = runSimulation(sc, controller, cfg, opts)
%     sc          scenario struct from scenarioDefinitions
%     controller  'B0' A* + waypoint tracking (no avoidance, no replanning)
%                 'B1' A* + proximity-based stop (no replanning)
%                 'P'  A* + prediction + DWA-style local planner + replanning
%     cfg         defaultConfig() (scenario overrides are applied here)
%     opts.seed   random seed for sensor noise (default sc.seed)
%     opts.verbose (default false), opts.logScans (default cfg.log.scans)
%
%   Per step (identical in the Simulink model, see simulink/):
%     simSense    world advance, LiDAR, tracking, local map
%     simReplan   blockage detection, debounced A* replanning (P only)
%     simCommand  look-ahead tracking + local safety layer (B1 stop / P DWA)
%     limitCommand, differentialDriveStep (plant), simPostStep (truth checks, log)
%   Ground truth (world.truth, world.dyn, robot.pose) is used ONLY by the
%   simulator (sensor model, collision check, metrics). Navigation uses
%   the prior map world.known, LiDAR returns and the estimated pose.
    if nargin < 3 || isempty(cfg), cfg = defaultConfig(); end
    if nargin < 4, opts = struct(); end
    ctx = simInit(sc, controller, cfg, opts);
    dt = ctx.cfg.dt;
    for k = 1:ctx.N
        if ctx.done, break; end
        t0 = tic;
        t = (k - 1) * dt;
        ctx = simSense(ctx, t);
        ctx = simReplan(ctx, t);
        [ctx, cmd] = simCommand(ctx);
        cmd = limitCommand(cmd, ctx.robot.cmd, ctx.cfg);
        ctx.robot = differentialDriveStep(ctx.robot, cmd, dt, ctx.cfg);
        ctx = simPostStep(ctx, k, t, cmd, toc(t0));
    end
    [result, log] = simFinalize(ctx);
end
