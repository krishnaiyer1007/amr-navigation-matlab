function [idx, dist] = closestPathIndex(path, p, prevIdx)
%CLOSESTPATHINDEX Closest waypoint, searched in a forward window from prevIdx.
%   Prevents jumping back along a path that loops near itself.
    n = size(path, 1);
    if nargin < 3 || isempty(prevIdx) || prevIdx < 1, prevIdx = 1; end
    prevIdx = min(prevIdx, n);
    lo = max(1, prevIdx - 5); hi = min(n, prevIdx + 60);
    d2 = (path(lo:hi,1) - p(1)).^2 + (path(lo:hi,2) - p(2)).^2;
    [m, k] = min(d2);
    idx = lo + k - 1; dist = sqrt(m);
    if dist > 1.0          % lost the window: global search
        d2 = (path(:,1) - p(1)).^2 + (path(:,2) - p(2)).^2;
        [m, idx] = min(d2); dist = sqrt(m);
    end
end
