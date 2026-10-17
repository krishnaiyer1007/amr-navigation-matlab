function S = summarizeResults(results, groupField)
%SUMMARIZERESULTS Aggregate metrics per group (default: controller).
%   Completion time statistics use SUCCESSFUL runs only (failed runs are
%   counted separately, never averaged as if they finished).
    if nargin < 2, groupField = 'controller'; end
    g = {results.(groupField)};
    if isnumeric(g{1}), g = cellfun(@num2str, g, 'UniformOutput', false); end
    keys = unique(g, 'stable');
    S = struct([]);
    for i = 1:numel(keys)
        R = results(strcmp(g, keys{i}));
        ok = [R.success];
        tt = [R(ok).time];
        s = struct();
        s.group = keys{i};
        s.runs = numel(R);
        s.successes = nnz(ok);
        s.successRate = 100 * nnz(ok) / numel(R);
        s.collisionRate = 100 * nnz([R.collisions] > 0) / numel(R);
        reasons = {R.reason};
        s.timeouts = nnz(strcmp(reasons, 'timeout'));
        s.stalls = nnz(strcmp(reasons, 'stall'));
        s.unreachable = nnz(strcmp(reasons, 'unreachable'));
        s.timeMean = mean(tt); s.timeStd = std(tt); s.timeMedian = median(tt); s.timeWorst = max([tt NaN]);
        if isempty(tt), s.timeMean = NaN; s.timeStd = NaN; s.timeMedian = NaN; s.timeWorst = NaN; end
        L = [R(ok).pathLength];
        s.pathLengthMean = mean([L NaN(1, isempty(L))]); s.pathLengthStd = std(L);
        if isempty(L), s.pathLengthStd = NaN; end
        s.efficiencyMean = mean([R(ok).efficiency]);
        if ~any(ok), s.efficiencyMean = NaN; end
        s.minClearanceMean = mean([R.minClearance]);
        s.minClearanceWorst = min([R.minClearance]);
        s.trackErrMean = mean([R.meanTrackErr]);
        s.replansMean = mean([R.replans]);
        lat = [R.replanLatencyMax]; lat = lat(~isnan(lat));
        s.replanLatencyWorst = max([lat NaN]);
        s.stepTimeMeanMs = 1000 * mean([R.stepTimeMean]);
        s.planTimeMeanMs = 1000 * mean([R.planTimeMean]);
        s.controlEffortMean = mean([R.controlEffort]);
        S = [S s]; %#ok<AGROW>
    end
end
