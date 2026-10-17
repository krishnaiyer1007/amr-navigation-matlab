function [cmd, info] = computeTrackingCommand(pose, path, prevIdx, cfg, goal)
%COMPUTETRACKINGCOMMAND Look-ahead heading controller for path following.
%   e_theta = wrap(atan2(yt - y, xt - x) - theta)
%   w = Kp * e_theta,  v = vMax * max(0, cos(e_theta)), slowed near the goal.
%   Returns info.closestIdx (monotone search from prevIdx), info.target,
%   info.crossTrack (distance to the path) and info.headingErr.
    info = struct('closestIdx', 1, 'target', pose(1:2), 'crossTrack', 0, ...
                  'headingErr', 0, 'lookIdx', 1);
    if isempty(path)
        cmd = [0 0]; return;
    end
    [ci, ct] = closestPathIndex(path, pose(1:2), prevIdx);
    % look-ahead point by arc length
    L = cfg.control.lookahead;
    n = size(path, 1);
    li = ci; acc = 0;
    while li < n && acc < L
        acc = acc + norm(path(li+1,:) - path(li,:));
        li = li + 1;
    end
    tgt = path(li, :);
    e = wrapAngle(atan2(tgt(2) - pose(2), tgt(1) - pose(1)) - pose(3));
    w = cfg.control.KpHeading * e;
    v = cfg.robot.vMax * max(0, cos(e));
    if abs(e) > pi/2, v = 0; end
    dGoal = norm(goal(1:2) - pose(1:2));
    v = min(v, cfg.control.goalSlowdown * dGoal + 0.05);
    cmd = [v, min(max(w, -cfg.robot.wMax), cfg.robot.wMax)];
    info.closestIdx = ci; info.target = tgt; info.crossTrack = ct;
    info.headingErr = e; info.lookIdx = li;
end
