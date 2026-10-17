function p = smoothPath(path, occ, res, spacing)
%SMOOTHPATH Line-of-sight shortcutting followed by uniform resampling.
%   Removes the grid staircase while staying collision free on OCC (the
%   inflated grid used for planning). Returns waypoints spaced SPACING m.
%   OCC may be stricter than the planning grid (e.g. including a soft
%   clearance band): segments that cannot be shortcut keep the original
%   A* waypoints, so smoothing never removes a feasible path.
    if nargin < 4, spacing = 0.1; end
    if size(path, 1) <= 2
        p = resamplePath(path, spacing);
        return;
    end
    keep = path(1,:);
    i = 1; n = size(path, 1);
    while i < n
        j = n;
        while j > i + 1 && ~hasLineOfSight(occ, path(i,:), path(j,:), res)
            j = j - 1;
        end
        keep(end+1,:) = path(j,:); %#ok<AGROW>
        i = j;
    end
    p = resamplePath(keep, spacing);
end
