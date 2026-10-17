% testAvoidance  DWA-style local planner safety properties.
cfg = defaultConfig();
path = [(1:0.1:9)' 5 * ones(81, 1)];

%% Candidates respect dynamic limits
st = struct('pose', [1 5 0], 'v', 0.3, 'w', 0.2);
[cmd, info] = selectSafeVelocity(st, [0.65 0], path, 1, zeros(0,2), zeros(0,5), cfg);
T = cfg.dwa.reachTime;
assert(cmd(1) <= min(cfg.robot.vMax, 0.3 + cfg.robot.aMax * T) + 1e-9);
assert(cmd(1) >= max(0, 0.3 - cfg.robot.aMax * T) - 1e-9);
assert(abs(cmd(2) - 0.2) <= cfg.robot.alphaMax * T + 1e-9);
assert(info.mode == 0 && info.nSafe == info.nCand, 'Free space should be all safe');

%% Free space: drives forward along the path
st = struct('pose', [1 5 0], 'v', 0.65, 'w', 0);
cmd = selectSafeVelocity(st, [0.65 0], path, 1, zeros(0,2), zeros(0,5), cfg);
assert(cmd(1) > 0.5 && abs(cmd(2)) < 0.3);

%% Wall straight ahead: candidates that would hit it are rejected
wall = [3.0 * ones(41,1), linspace(3, 7, 41)'];
st = struct('pose', [2.3 5 0], 'v', 0.65, 'w', 0);
[cmd, info] = selectSafeVelocity(st, [0.65 0], path, 1, wall, zeros(0,5), cfg);
assert(info.nSafe < info.nCand, 'Unsafe candidates not rejected');
% simulate chosen command for commit time then brake: must stay clear
x = 2.3; v = cmd(1);
x = x + v * cfg.dwa.commitTime; x = x + v^2 / (2 * cfg.robot.aMax);
assert(x + cfg.robot.radius < 3.0 || info.mode == 2, 'Selected command reaches the wall');

%% Moving obstacle on a collision course is predicted and avoided
obs = [4.2 4.0 0 0.5 0.3];           % will cross y = 5 just ahead of the robot
st = struct('pose', [3 5 0], 'v', 0.65, 'w', 0);
[cmd, info] = selectSafeVelocity(st, [0.65 0], path, 21, zeros(0,2), obs, cfg);
cmdNo = selectSafeVelocity(st, [0.65 0], path, 21, zeros(0,2), zeros(0,5), cfg);
assert(info.nSafe < info.nCand, 'Prediction did not reject any candidate');
assert(~isequal(cmd, cmdNo), 'Moving obstacle did not change the command');

%% Surrounded with no safe motion -> controlled stop
ring = [2 + 0.3 * cos(linspace(0, 2*pi, 60))', 5 + 0.3 * sin(linspace(0, 2*pi, 60))'];
st = struct('pose', [2 5 0], 'v', 0.4, 'w', 0);
[cmd, info] = selectSafeVelocity(st, [0.65 0], path, 11, ring, zeros(0,5), cfg);
assert(info.mode == 2, 'Expected stop mode');
assert(cmd(1) < 0.4, 'Stop command must decelerate');

%% Proximity stop baseline (B1) halts in front of an obstacle
tr = initializeTracks();
st = struct('pose', [2 5 0], 'v', 0.5, 'w', 0);
[c1, stopped] = proximityStop([0.5 0], st, [2.6 5], tr, cfg);
assert(stopped && c1(1) == 0);
[~, stopped] = proximityStop([0.5 0], st, [2.6 6.5], tr, cfg);
assert(~stopped);

%% Oncoming obstacle with no safe forward command -> evasive escape, not freezing
cfg = defaultConfig();
path = [(1:0.1:9)' 5 * ones(81, 1)];
obs = [3.6 5 -0.5 0 0.3];               % head-on, 1.6 m away, closing at 0.5 m/s
st = struct('pose', [2 5 0], 'v', 0, 'w', 0);
[cmd, info] = selectSafeVelocity(st, [0.65 0], path, 11, zeros(0,2), obs, cfg);
assert(info.mode == 1, 'Expected evasive mode, got %d', info.mode);
assert(cmd(1) < 0 || abs(cmd(2)) > 0.1, 'Robot froze in front of the obstacle');
assert(cmd(1) >= cfg.robot.vMin - 1e-12, 'Reverse speed limit violated');
