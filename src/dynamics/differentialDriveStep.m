function robot = differentialDriveStep(robot, cmd, dt, cfg)
%DIFFERENTIALDRIVESTEP Advance the differential-drive plant by one step.
%   cmd = [v w] body velocity request (already limited by limitCommand).
%   1) inverse kinematics -> wheel speed targets
%   2) wheel speed PI(D) control on first-order motor dynamics (or ideal)
%   3) forward kinematics v = r/2 (wR + wL), w = r/b (wR - wL)
%   4) pose integration (forward Euler or midpoint), yaw wrapped to [-pi, pi)
%   5) odometry: poseEst from (optionally noisy) wheel speeds
    r = cfg.robot.wheelRadius; b = cfg.robot.wheelBase;
    target = [(cmd(1) + cmd(2) * b / 2) / r, (cmd(1) - cmd(2) * b / 2) / r];

    if cfg.robot.idealWheels
        wheel = target;
    else
        [u, robot.pid] = wheelPID(target, robot.wheel, robot.pid, dt, cfg);
        tau = cfg.robot.motorTau;
        a = 1 - exp(-dt / tau);                       % exact ZOH discretisation
        wheel = robot.wheel + a * (u - robot.wheel);  % first-order motor
        wheel = min(max(wheel, -cfg.robot.wheelSpeedMax), cfg.robot.wheelSpeedMax);
    end
    robot.wheel = wheel;

    v = r / 2 * (wheel(1) + wheel(2));
    w = r / b * (wheel(1) - wheel(2));
    robot.pose = integratePose(robot.pose, v, w, dt, cfg.robot.integrator);
    robot.v = v; robot.w = w;
    robot.cmd = cmd;

    % odometry estimate
    if cfg.odom.useTruePose
        robot.poseEst = robot.pose;
    else
        n = 1 + cfg.odom.wheelNoiseStd * randn(1, 2);
        wm = wheel .* n;
        ve = r / 2 * (wm(1) + wm(2));
        we = r / b * (wm(1) - wm(2));
        robot.poseEst = integratePose(robot.poseEst, ve, we, dt, cfg.robot.integrator);
    end
end

function p = integratePose(p, v, w, dt, method)
    switch method
        case 'midpoint'
            thm = p(3) + 0.5 * w * dt;
            p = [p(1) + v * cos(thm) * dt, p(2) + v * sin(thm) * dt, p(3) + w * dt];
        otherwise   % forward Euler
            p = [p(1) + v * cos(p(3)) * dt, p(2) + v * sin(p(3)) * dt, p(3) + w * dt];
    end
    p(3) = wrapAngle(p(3));
end
