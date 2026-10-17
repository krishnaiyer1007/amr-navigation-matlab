function ok = hasLineOfSight(occ, p1, p2, res)
%HASLINEOFSIGHT True if the straight segment p1->p2 crosses no blocked cell.
%   Samples the segment at quarter-cell spacing (conservative supercover).
    [ny, nx] = size(occ);
    L = norm(p2 - p1);
    n = max(2, ceil(L * res * 4) + 1);
    s = linspace(0, 1, n)';
    pts = p1 + s .* (p2 - p1);
    [r, c, inside] = world2grid(pts, res, ny, nx);
    ok = all(inside) && ~any(occ(sub2ind([ny nx], r, c)));
end
