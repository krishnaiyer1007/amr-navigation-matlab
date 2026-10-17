function [pts, beamIdx] = scanToPoints(scan, pose)
%SCANTOPOINTS Convert valid returns to world-frame points using POSE.
%   POSE should be the ESTIMATED pose (what the robot believes).
    v = find(scan.valid);
    a = pose(3) + scan.angles(v);
    rr = scan.ranges(v);
    pts = [pose(1) + rr(:) .* cos(a(:)), pose(2) + rr(:) .* sin(a(:))];
    beamIdx = v(:);
end
