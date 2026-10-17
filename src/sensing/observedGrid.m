function [occ, pts] = observedGrid(lm)
%OBSERVEDGRID Logical grid and cell-centre points of currently observed obstacles.
    occ = isfinite(lm.lastSeen) & lm.hits >= lm.minHits;
    [r, c] = find(occ);
    pts = grid2world(r, c, lm.res);
end
