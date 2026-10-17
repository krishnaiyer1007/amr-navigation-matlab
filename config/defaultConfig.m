function cfg = defaultConfig()
%DEFAULTCONFIG Central parameter set for the AMR navigation project.
%   cfg = defaultConfig() returns a struct of tunable starting values.
%   All units are SI (m, s, rad) unless stated otherwise.
%   Record any calibrated changes and the reason in docs/testPlan.md.

    % ---------------- Simulation ----------------
    cfg.dt       = 0.05;     % s, simulation/control step
    cfg.duration = 120;      % s, maximum scenario duration (timeout)
    cfg.seed     = 42;       % default random seed

    % ---------------- Map ----------------
    cfg.map.width      = 20;   % m
    cfg.map.height     = 15;   % m
    cfg.map.resolution = 10;   % cells per metre
    % Grid convention: grid(row, col), row 1 is the BOTTOM (y = 0) row.
    % Cell (r,c) centre: x = (c-0.5)/res, y = (r-0.5)/res.

    % ---------------- Robot ----------------
    cfg.robot.radius      = 0.22;  % m, circular footprint approximation
    cfg.robot.wheelRadius = 0.05;  % m
    cfg.robot.wheelBase   = 0.32;  % m, wheel centre to wheel centre
    cfg.robot.vMax        = 0.65;  % m/s
    cfg.robot.vMin        = -0.2;  % m/s, slow reverse allowed ONLY for evasive escapes (rear is a LiDAR blind zone)
    cfg.robot.wMax        = 1.4;   % rad/s
    cfg.robot.aMax        = 0.6;   % m/s^2
    cfg.robot.alphaMax    = 2.0;   % rad/s^2
    cfg.robot.motorTau    = 0.06;  % s, first order wheel motor time constant
    cfg.robot.wheelSpeedMax = 25;  % rad/s, motor saturation
    cfg.robot.idealWheels = false; % true: wheels follow command instantly
    cfg.robot.integrator  = 'euler'; % 'euler' or 'midpoint'

    % ---------------- Wheel PI controller (with feedforward) ----------------
    cfg.wheelPID.Kp   = 0.5;
    cfg.wheelPID.Ki   = 3.0;
    cfg.wheelPID.Kd   = 0.0;
    cfg.wheelPID.iMax = 10;        % rad/s, anti-windup clamp on integral term

    % ---------------- Odometry / state estimate ----------------
    cfg.odom.useTruePose = true;   % Phase 1-4 baseline: exact pose
    cfg.odom.wheelNoiseStd = 0.02; % fractional wheel speed noise when false

    % ---------------- Safety ----------------
    cfg.safety.margin     = 0.15;  % m, clearance target used for inflation
    cfg.safety.hardMargin = 0.05;  % m, DWA hard rejection clearance
    cfg.safety.ttcStop    = 0.7;   % s, conservative stop threshold (B1)
    cfg.safety.b1StopDist = 0.25;  % m, extra B1 stop distance beyond braking

    % ---------------- LiDAR ----------------
    cfg.sensor.maxRange      = 6.0;  % m
    cfg.sensor.fov           = 270;  % degrees
    cfg.sensor.numBeams      = 181;
    cfg.sensor.rangeNoiseStd = 0.02; % m
    cfg.sensor.missProb      = 0.0;  % probability of a missed return
    cfg.sensor.falseProb     = 0.0;  % probability of a spurious short return

    % ---------------- Local map and tracking ----------------
    cfg.localMap.memory       = 15;   % s, observed obstacle cells expire after this
    cfg.localMap.staticTol    = 0.15; % m, min tolerance: point this close to known map is "known"
    cfg.localMap.noiseGate    = 2.5;  % tolerance = max(staticTol, noiseGate * rangeNoiseStd)
    cfg.localMap.minHits      = 2;    % returns needed before a cell counts as occupied
    cfg.track.clusterDist     = 0.30; % m, clustering distance (+3*rangeNoiseStd)
    cfg.track.mergeDist       = 0.45; % m, tracks closer than this are merged
    cfg.track.gate            = 0.8;  % m, association gate
    cfg.track.alpha           = 0.35; % position filter gain
    cfg.track.beta            = 0.06; % velocity filter gain (low: 20 Hz updates amplify noise)
    cfg.track.maxAge          = 1.0;  % s, expire unseen tracks
    cfg.track.movingSpeed     = 0.12; % m/s, above this a track is "moving"
    cfg.track.minHits         = 5;    % hits before velocity is trusted
    cfg.track.maxRadius       = 0.8;  % m, larger clusters are treated as static structure

    % ---------------- Global planner ----------------
    cfg.planner.replanPeriod      = 2.0;  % s, periodic replan (only if map changed)
    cfg.planner.minReplanInterval = 0.5;  % s, debounce between replans
    cfg.planner.blockDebounce     = 2;    % consecutive blocked steps before replan
    cfg.planner.checkAhead        = 8.0;  % m of remaining path checked for blockage (> sensor range)
    cfg.planner.costBand          = 0.35; % m, soft-cost band beyond hard inflation
    cfg.planner.costWeight        = 1.5;  % extra cost factor inside the band
    cfg.planner.deviationReplan   = 1.0;  % m, cross track error that forces replan
    cfg.planner.smooth            = true;
    cfg.planner.waypointSpacing   = 0.10; % m, path resampling

    % ---------------- Tracking controller ----------------
    cfg.control.lookahead  = 0.6;   % m
    cfg.control.KpHeading  = 2.0;
    cfg.control.goalSlowdown = 0.8; % 1/s, v <= k * distance to goal

    % ---------------- DWA style local planner ----------------
    cfg.dwa.nV          = 7;     % velocity samples
    cfg.dwa.nW          = 15;    % yaw rate samples
    cfg.dwa.reachTime   = 0.4;   % s, window of reachable velocities
    cfg.dwa.horizon     = 2.5;   % s, prediction horizon
    cfg.dwa.simDt       = 0.1;   % s, rollout step
    cfg.dwa.commitTime  = 0.5;   % s, command held before braking check
    cfg.dwa.goalTime    = 1.5;   % s, progress evaluation time
    cfg.dwa.unc0        = 0.05;  % m, obstacle prediction uncertainty at t=0
    cfg.dwa.uncRate     = 0.10;  % m/s, uncertainty growth
    cfg.dwa.maxPoints   = 250;   % static scan points used per step
    cfg.dwa.considerDist = 3.5;  % m, ignore points further than this
    cfg.dwa.wGoal   = 1.0;
    cfg.dwa.wPath   = 0.8;
    cfg.dwa.wClear  = 1.0;
    cfg.dwa.wSmooth = 0.1;
    cfg.dwa.wSpeed  = 0.3;
    cfg.dwa.clearSoft = 0.5;     % m, clearance below which cost rises
    cfg.dwa.targetDist = 1.2;    % m, local target distance along the path

    % ---------------- Scripted moving obstacles (ground truth) ----------------
    cfg.obstacles.accel = 0.5;    % m/s^2, max speed change (no instant reversals)
    cfg.obstacles.dwell = 1.0;    % s, pause at each end of a patrol route

    % ---------------- Goal and failure ----------------
    cfg.goal.tolerance = 0.25;    % m
    cfg.fail.stallTime = 25;      % s without 0.3 m progress -> 'stall'
    cfg.fail.unreachableHold = 3; % s stopped with no valid path -> 'unreachable'

    % ---------------- Logging ----------------
    cfg.log.scans = false;        % store every scan (memory heavy, needed for animation)
end
