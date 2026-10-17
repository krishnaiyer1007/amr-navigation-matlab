function [tr, isMovingPt, cl, inCluster] = estimateObstacleMotion(tr, pts, beamIdx, isNew, robotXY, t, cfg)
%ESTIMATEOBSTACLEMOTION Cluster new returns, associate to tracks, filter velocity.
%   - Only returns NOT explained by the known map are considered.
%   - Nearest-neighbour association with a gate (cfg.track.gate) against
%     constant-velocity predictions.
%   - Alpha-beta filter on position and velocity (gains alpha, beta).
%   - Tracks unseen for more than maxAge seconds expire (occlusion).
%   - A track is MOVING when it has >= minHits hits and speed > movingSpeed.
%   Output isMovingPt flags points that belong to a moving track; those are
%   excluded from the static local map. inCluster flags new points that
%   belong to a cluster of >= 2 points (isolated returns are noise).
    T = cfg.track;
    isMovingPt = false(size(pts, 1), 1);
    newIdx = find(isNew);
    cl = clusterScan(pts(newIdx, :), beamIdx(newIdx), robotXY, cfg);
    inCluster = false(size(pts, 1), 1);
    for k = 1:numel(cl)
        cl(k).idx = newIdx(cl(k).idx);
        inCluster(cl(k).idx) = true;
    end
    small = find(~[cl.large]);

    L = tr.list;
    nT = numel(L);
    assigned = zeros(1, numel(cl));          % cluster -> track index
    if nT > 0 && ~isempty(small)
        pred = zeros(nT, 2);
        for i = 1:nT, pred(i,:) = L(i).pos + L(i).vel * (t - L(i).lastSeen); end
        C = reshape([cl(small).c], 2, [])';
        D = sqrt((pred(:,1) - C(:,1)').^2 + (pred(:,2) - C(:,2)').^2);
        D(D > T.gate) = inf;
        while true
            [m, k] = min(D(:));
            if ~isfinite(m), break; end
            [i, j] = ind2sub(size(D), k);
            assigned(small(j)) = i;
            D(i, :) = inf; D(:, j) = inf;
        end
    end

    for j = small
        z = cl(j).c;
        i = assigned(j);
        if i > 0
            dt = max(t - L(i).lastSeen, 1e-3);
            p = L(i).pos + L(i).vel * dt;
            res = z - p;
            L(i).pos = p + T.alpha * res;
            L(i).vel = L(i).vel + (T.beta / dt) * res;
            sp = norm(L(i).vel);
            if sp > 2.0, L(i).vel = L(i).vel / sp * 2.0; end
            L(i).r = 0.7 * L(i).r + 0.3 * cl(j).r;
            L(i).lastSeen = t;
            L(i).hits = L(i).hits + 1;
        else
            L(end+1) = struct('id', tr.nextId, 'pos', z, 'vel', [0 0], 'r', cl(j).r, ...
                              'lastSeen', t, 'hits', 1, 'moving', false); %#ok<AGROW>
            tr.nextId = tr.nextId + 1;
            i = numel(L);
            assigned(j) = i;
        end
    end

    % merge duplicate tracks (one object split into two clusters)
    keep = true(1, numel(L));
    for i = 1:numel(L)
        for j = i+1:numel(L)
            if keep(i) && keep(j) && norm(L(i).pos - L(j).pos) < T.mergeDist
                if L(j).hits > L(i).hits, keep(i) = false; else, keep(j) = false; end
            end
        end
    end
    % expire stale tracks, update moving flag
    for i = 1:numel(L)
        if t - L(i).lastSeen > T.maxAge, keep(i) = false; end
        wasMoving = L(i).moving;
        sp = norm(L(i).vel);
        % hysteresis: once moving, stay moving until clearly stopped
        L(i).moving = L(i).hits >= T.minHits && ...
            (sp > T.movingSpeed || (wasMoving && sp > 0.5 * T.movingSpeed));
    end
    for j = small
        i = assigned(j);
        if i > 0 && L(i).moving, isMovingPt(cl(j).idx) = true; end
    end
    tr.list = L(keep);
end
