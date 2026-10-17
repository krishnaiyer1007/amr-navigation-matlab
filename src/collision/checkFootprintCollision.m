function [collided, clearance] = checkFootprintCollision(pose, world, cfg)
%CHECKFOOTPRINTCOLLISION Ground-truth collision and clearance check.
%   Circular footprint of radius cfg.robot.radius vs. occupied cells
%   (treated as squares) of world.truth and dynamic obstacle circles.
%   clearance = minimum surface-to-surface distance (m), capped at 1.0 m
%   for static obstacles beyond the local search window.
%   collided = clearance < 0 (overlap). Touching (clearance == 0) is not a collision.
    R = cfg.robot.radius;
    win = 1.0 + R;
    res = world.res; h = 0.5 / res;
    c0 = max(1, floor((pose(1) - win) * res) + 1); c1 = min(world.nx, floor((pose(1) + win) * res) + 1);
    r0 = max(1, floor((pose(2) - win) * res) + 1); r1 = min(world.ny, floor((pose(2) + win) * res) + 1);
    sub = world.truth(r0:r1, c0:c1);
    clearance = 1.0;
    if any(sub(:))
        [rr, cc] = find(sub);
        cx = (cc + c0 - 1 - 0.5) / res; cy = (rr + r0 - 1 - 0.5) / res;
        dx = max(abs(pose(1) - cx) - h, 0); dy = max(abs(pose(2) - cy) - h, 0);
        clearance = min(clearance, min(sqrt(dx.^2 + dy.^2)) - R);
    end
    % outside the map counts as occupied
    edge = min([pose(1), pose(2), world.width - pose(1), world.height - pose(2)]) - R;
    clearance = min(clearance, edge);
    for i = 1:numel(world.dyn)
        d = norm(pose(1:2) - world.dyn(i).pos) - world.dyn(i).r - R;
        clearance = min(clearance, d);
    end
    collided = clearance < 0;
end
