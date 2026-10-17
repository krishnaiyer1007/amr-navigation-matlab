function [r, c, inside] = world2grid(xy, res, ny, nx)
%WORLD2GRID Convert world coordinates (m) to grid (row, col).
%   Row 1 is the bottom row (y = 0). Cell (r,c) spans
%   x in [(c-1)/res, c/res), y in [(r-1)/res, r/res).
%   Indices are clamped to the grid; INSIDE flags points within bounds.
    c = floor(xy(:,1) * res) + 1;
    r = floor(xy(:,2) * res) + 1;
    inside = r >= 1 & r <= ny & c >= 1 & c <= nx;
    r = min(max(r, 1), ny);
    c = min(max(c, 1), nx);
end
