function robot = initializeRobot(cfg, startPose)
%INITIALIZEROBOT Robot state struct.
%   pose      true pose [x y theta] (ground truth, simulator only)
%   poseEst   estimated pose used by navigation (odometry or true pose)
%   wheel     actual wheel angular speeds [wR wL] (rad/s)
%   pid       wheel controller state
%   cmd       last limited body command [v w]
    robot.pose = startPose(:)';
    robot.pose(3) = wrapAngle(robot.pose(3));
    robot.poseEst = robot.pose;
    robot.wheel = [0 0];
    robot.v = 0; robot.w = 0;
    robot.pid = struct('integ', [0 0], 'prevErr', [0 0]);
    robot.cmd = [0 0];
    robot.cfg = cfg.robot; %#ok<STRNU>
    robot = rmfield(robot, 'cfg');
end
