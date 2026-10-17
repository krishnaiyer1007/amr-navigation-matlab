function L = dijkstraRef(occ, s, g)
%DIJKSTRAREF Brute-force Dijkstra reference used by testAStar (same move rules).
    [ny, nx] = size(occ); N = ny * nx;
    d = inf(N, 1); done = false(N, 1);
    si = sub2ind([ny nx], s(1), s(2)); gi = sub2ind([ny nx], g(1), g(2));
    d(si) = 0;
    nb = [1 0 1; -1 0 1; 0 1 1; 0 -1 1; 1 1 sqrt(2); 1 -1 sqrt(2); -1 1 sqrt(2); -1 -1 sqrt(2)];
    while true
        dd = d; dd(done) = inf; [m, u] = min(dd);
        if ~isfinite(m) || u == gi, break; end
        done(u) = true; [r, c] = ind2sub([ny nx], u);
        for k = 1:8
            r2 = r + nb(k,1); c2 = c + nb(k,2);
            if r2 < 1 || r2 > ny || c2 < 1 || c2 > nx || occ(r2, c2), continue; end
            if k > 4 && (occ(r + nb(k,1), c) || occ(r, c2)), continue; end
            v = sub2ind([ny nx], r2, c2);
            d(v) = min(d(v), d(u) + nb(k,3));
        end
    end
    L = d(gi);
end
