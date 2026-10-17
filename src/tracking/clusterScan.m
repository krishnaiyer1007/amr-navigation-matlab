function cl = clusterScan(pts, beamIdx, robotXY, cfg)
%CLUSTERSCAN Group consecutive beam returns into obstacle clusters.
%   Consecutive returns (in beam order) closer than clusterDist belong to
%   the same cluster. Single-point clusters are discarded as noise.
%   Each cluster: centre (corrected for the visible-arc bias), radius
%   estimate, member point indices and a flag for large (wall-like) objects.
    cl = struct('c', {}, 'r', {}, 'idx', {}, 'large', {});
    n = size(pts, 1);
    if n == 0, return; end
    cd = cfg.track.clusterDist + 3 * cfg.sensor.rangeNoiseStd;
    gap = [true; sqrt(sum(diff(pts).^2, 2)) > cd | diff(beamIdx(:)) > 3];
    starts = find(gap); ends = [starts(2:end) - 1; n];
    for k = 1:numel(starts)
        id = (starts(k):ends(k))';
        if numel(id) < 2, continue; end
        P = pts(id, :);
        ext = norm(P(end,:) - P(1,:));
        rEst = min(max(ext / 2, 0.1), cfg.track.maxRadius);
        m = mean(P, 1);
        u = m - robotXY; nu = norm(u);
        if nu > 1e-6, m = m + u / nu * 0.6 * rEst; end
        cl(end+1) = struct('c', m, 'r', rEst, 'idx', id, ...
                           'large', ext > 2 * cfg.track.maxRadius); %#ok<AGROW>
    end
end
