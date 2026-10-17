function lm = updateLocalMap(lm, scan, pose, pts, beamIdx, isNew, isMoving, t, cfg) %#ok<INUSL>
%UPDATELOCALMAP Update the observed-obstacle grid from one scan.
%   1) Free-space clearing: cells traversed by a valid beam (up to 0.2 m
%      before its return) are cleared, so obstacles that moved away fade.
%      Beams with no return are NOT used for clearing - unknown space is
%      never silently treated as proven free.
%   2) Returns that are new (not explained by the known map), belong to a
%      cluster of >= 2 points (isNew is expected to be cluster-filtered)
%      and are not associated with a moving track are stamped with time t
%      and their hit count is incremented. A cell is OCCUPIED only after
%      localMap.minHits returns (evidence accumulation rejects spurious
%      single returns).
%   3) Cells older than localMap.memory seconds expire.
    res = lm.res; ny = lm.ny; nx = lm.nx;
    v = find(scan.valid);
    if ~isempty(v)
        step = 0.5 / res;
        rr = scan.ranges(v) - 0.2;
        maxS = max(rr);
        if maxS > step
            s = (step:step:maxS)';
            a = pose(3) + scan.angles(v);
            X = pose(1) + s * cos(a); Y = pose(2) + s * sin(a);
            m = s <= rr;                                  % implicit expansion
            c = floor(X(m) * res) + 1; r = floor(Y(m) * res) + 1;
            ok = r >= 1 & r <= ny & c >= 1 & c <= nx;
            idx = r(ok) + (c(ok) - 1) * ny;
            lm.lastSeen(idx) = -inf;
            lm.hits(idx) = 0;
        end
    end
    stamp = isNew & ~isMoving;
    if any(stamp)
        P = pts(stamp, :);
        c = floor(P(:,1) * res) + 1; r = floor(P(:,2) * res) + 1;
        ok = r >= 1 & r <= ny & c >= 1 & c <= nx;
        idx = unique(r(ok) + (c(ok) - 1) * ny);
        lm.lastSeen(idx) = t;
        lm.hits(idx) = min(lm.hits(idx) + 1, 10);
    end
    old = lm.lastSeen < t - cfg.localMap.memory;
    lm.lastSeen(old) = -inf;
    lm.hits(old) = 0;
    cnt = nnz(lm.hits >= lm.minHits);
    if cnt ~= lm.occCount
        lm.version = lm.version + 1;
        lm.occCount = cnt;
    end
end
