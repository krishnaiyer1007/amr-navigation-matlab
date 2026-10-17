function result = computeMetrics(result, log, world, cfg) %#ok<INUSL>
%COMPUTEMETRICS Fill performance metrics from a run log (README section 13).
%   time            simulated seconds at goal (NaN if unsuccessful)
%   pathLength      sum of distances between successive robot positions
%   efficiency      reference (initial A* smoothed) length / actual length
%   meanTrackErr    mean cross-track distance to the active reference path
%   minClearance    minimum ground-truth surface distance (static capped at 1 m)
%   controlEffort   sum of squared command changes (dv^2 + dw^2), a proxy
%   stopTime        seconds spent in controlled stop mode
%   evasiveTime     seconds spent in evasive mode (no fully safe candidate)
    dt = cfg.dt;
    if isempty(log.t)
        return;
    end
    P = log.pose(:, 1:2);
    seg = sqrt(sum(diff([world.start(1:2); P]).^2, 2));
    result.pathLength = sum(seg);
    if result.success
        result.time = log.t(end);
        result.efficiency = result.refLength / max(result.pathLength, eps);
    end
    result.meanTrackErr = mean(log.crossTrack(~isnan(log.crossTrack)));
    result.minClearance = min(log.clearance);
    dc = diff([0 0; log.cmd]);
    result.controlEffort = sum(dc(:,1).^2 + dc(:,2).^2);
    result.stepTimeMean = mean(log.stepTime);
    result.stepTimeMax = max(log.stepTime);
    result.stopTime = nnz(log.mode == 2) * dt;
    result.evasiveTime = nnz(log.mode == 1) * dt;
    result.finalDist = norm(P(end,:) - world.goal(1:2));
end
