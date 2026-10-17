function printSummary(S, filename)
%PRINTSUMMARY Print a summary table and optionally save it as CSV.
    fprintf('\n%-10s %5s %8s %8s %9s %9s %9s %9s %8s %8s %8s\n', 'Group', 'Runs', ...
        'Success%', 'Collis%', 'Tmean(s)', 'Tmed(s)', 'Tworst(s)', 'MinClr(m)', 'TrkErr', 'Replans', 'Step(ms)');
    for i = 1:numel(S)
        s = S(i);
        fprintf('%-10s %5d %8.1f %8.1f %9.1f %9.1f %9.1f %9.3f %8.3f %8.1f %8.2f\n', s.group, s.runs, ...
            s.successRate, s.collisionRate, s.timeMean, s.timeMedian, s.timeWorst, ...
            s.minClearanceWorst, s.trackErrMean, s.replansMean, s.stepTimeMeanMs);
    end
    if nargin > 1 && ~isempty(filename)
        f = fieldnames(S);
        data = cell(numel(S), numel(f));
        for i = 1:numel(S)
            for j = 1:numel(f), data{i,j} = S(i).(f{j}); end
        end
        writeCsv(filename, f', data);
    end
end
