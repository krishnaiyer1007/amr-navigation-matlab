function [blocked, firstIdx] = isPathBlocked(path, fromIdx, obsPts, clearance, checkAhead)
%ISPATHBLOCKED Check the next CHECKAHEAD metres of PATH against obstacles.
%   obsPts    Mx2 observed obstacle points (world) that are NOT in the
%             planner's current map (newly perceived static obstacles).
%   clearance minimum allowed distance from path to an obstacle point
%             (normally robot.radius + safety.margin).
%   Returns BLOCKED and the index of the first blocked waypoint.
    blocked = false; firstIdx = 0;
    if isempty(path) || isempty(obsPts), return; end
    n = size(path, 1);
    fromIdx = min(max(1, fromIdx), n);
    seg = sqrt(sum(diff(path(fromIdx:end,:)).^2, 2));
    lastIdx = fromIdx + find([0; cumsum(seg)] <= checkAhead, 1, 'last') - 1;
    P = path(fromIdx:lastIdx, :);
    % only consider points near the checked segment
    lo = min(P, [], 1) - clearance; hi = max(P, [], 1) + clearance;
    m = obsPts(:,1) >= lo(1) & obsPts(:,1) <= hi(1) & ...
        obsPts(:,2) >= lo(2) & obsPts(:,2) <= hi(2);
    Q = obsPts(m, :);
    if isempty(Q), return; end
    D2 = (P(:,1) - Q(:,1)').^2 + (P(:,2) - Q(:,2)').^2;
    hit = find(min(D2, [], 2) < clearance^2, 1, 'first');
    if ~isempty(hit)
        blocked = true;
        firstIdx = fromIdx + hit - 1;
    end
end
