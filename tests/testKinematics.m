% testKinematics  Differential-drive kinematics and integration checks.
% Script-based unit test: run with runtests('testKinematics') or run directly.
cfg = defaultConfig();
cfg.robot.idealWheels = true;      % isolate kinematics from motor dynamics
dt = cfg.dt; tol = 1e-9;

%% Zero wheel speed gives zero movement
r = initializeRobot(cfg, [1 2 0.3]);
for k = 1:50, r = differentialDriveStep(r, [0 0], dt, cfg); end
assert(norm(r.pose - [1 2 0.3]) < tol, 'Robot moved with zero command');

%% Equal wheel speeds produce straight motion
r = initializeRobot(cfg, [0 0 pi/6]);
for k = 1:100, r = differentialDriveStep(r, [0.5 0], dt, cfg); end
d = 0.5 * 100 * dt;
assert(norm(r.pose(1:2) - d * [cos(pi/6) sin(pi/6)]) < 1e-9, 'Straight motion wrong');
assert(abs(r.pose(3) - pi/6) < tol, 'Heading changed during straight motion');
assert(abs(r.wheel(1) - r.wheel(2)) < tol, 'Wheel speeds not equal');

%% Opposite wheel speeds rotate in place
r = initializeRobot(cfg, [3 3 0]);
for k = 1:40, r = differentialDriveStep(r, [0 1.0], dt, cfg); end
assert(norm(r.pose(1:2) - [3 3]) < tol, 'Translation during in-place rotation');
assert(abs(wrapAngle(r.pose(3) - 2.0)) < 1e-9, 'Wrong rotation angle');
assert(abs(r.wheel(1) + r.wheel(2)) < tol, 'Wheel speeds not opposite');

%% Wheel speed to body velocity mapping
wR = 10; wL = 6; rw = cfg.robot.wheelRadius; b = cfg.robot.wheelBase;
v = rw/2 * (wR + wL); w = rw/b * (wR - wL);
r = initializeRobot(cfg, [0 0 0]);
r = differentialDriveStep(r, [v w], dt, cfg);
assert(abs(r.wheel(1) - wR) < 1e-9 && abs(r.wheel(2) - wL) < 1e-9, 'Inverse kinematics wrong');
assert(abs(r.v - v) < 1e-12 && abs(r.w - w) < 1e-12, 'Forward kinematics wrong');

%% Circular motion: midpoint integration is more accurate than Euler
v = 0.5; w = 0.5; T = 2*pi / w; n = round(T / dt);
errs = zeros(1, 2); methods = {'euler', 'midpoint'};
for m = 1:2
    c = cfg; c.robot.integrator = methods{m};
    r = initializeRobot(c, [0 0 0]);
    for k = 1:n, r = differentialDriveStep(r, [v w], dt, c); end
    tEnd = n * dt;                       % analytic pose on circle radius v/w
    ex = v/w * sin(w * tEnd); ey = v/w * (1 - cos(w * tEnd));
    errs(m) = norm(r.pose(1:2) - [ex ey]);
end
assert(errs(2) < errs(1), 'Midpoint should beat Euler');
assert(errs(2) < 1e-3, 'Midpoint circle error too large');
fprintf('    circle closure error: Euler %.4f m, midpoint %.6f m\n', errs);

%% Yaw stays normalised to [-pi, pi)
r = initializeRobot(cfg, [0 0 3.1]);
for k = 1:200, r = differentialDriveStep(r, [0 1.4], dt, cfg); end
assert(r.pose(3) >= -pi && r.pose(3) < pi, 'Yaw not wrapped');

%% Limits: speed and acceleration saturation
cfg2 = defaultConfig();
c = limitCommand([5 5], [0 0], cfg2);
assert(abs(c(1) - cfg2.robot.aMax * cfg2.dt) < 1e-12, 'Acceleration limit not applied');
assert(abs(c(2) - cfg2.robot.alphaMax * cfg2.dt) < 1e-12, 'Angular acceleration limit not applied');
c = limitCommand([5 5], [0.65 1.4], cfg2);
assert(c(1) <= cfg2.robot.vMax + 1e-12 && c(2) <= cfg2.robot.wMax + 1e-12, 'Speed limit not applied');

%% Wheel PI loop converges to the target (motor dynamics on)
cfg2 = defaultConfig();
r = initializeRobot(cfg2, [0 0 0]);
for k = 1:60, r = differentialDriveStep(r, [0.5 0.5], cfg2.dt, cfg2); end
assert(abs(r.v - 0.5) < 0.01 && abs(r.w - 0.5) < 0.01, 'Wheel PI did not converge');
