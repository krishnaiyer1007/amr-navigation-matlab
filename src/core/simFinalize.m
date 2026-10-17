function [result, log] = simFinalize(ctx)
%SIMFINALIZE Trim the log and compute metrics.
    log = ctx.log; k = ctx.k;
    f = {'t', 'pose', 'poseEst', 'cmd', 'vel', 'wheel', 'clearance', 'crossTrack', ...
         'mode', 'nTracks', 'nMoving', 'blocked', 'stepTime'};
    for i = 1:numel(f), log.(f{i}) = log.(f{i})(1:k, :); end
    log.dynPos = log.dynPos(1:k, :, :);
    if isfield(log, 'scans'), log.scans = log.scans(1:k); end
    result = ctx.result;
    if ~isempty(ctx.latencies)
        result.replanLatencyMean = mean(ctx.latencies);
        result.replanLatencyMax = max(ctx.latencies);
    end
    result.planTimeMean = mean(ctx.planTimes);
    result = computeMetrics(result, log, ctx.world, ctx.cfg);
end
