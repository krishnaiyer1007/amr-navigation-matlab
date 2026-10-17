function saveResultsTable(results, filename)
%SAVERESULTSTABLE Write an array of run result structs to CSV.
    f = {'scenario', 'controller', 'seed', 'success', 'reason', 'time', 'pathLength', ...
         'refLength', 'efficiency', 'meanTrackErr', 'minClearance', 'collisions', ...
         'replans', 'replanLatencyMean', 'replanLatencyMax', 'stepTimeMean', ...
         'stepTimeMax', 'planTimeMean', 'controlEffort', 'stopTime', 'evasiveTime', 'finalDist'};
    extra = {};
    if isfield(results, 'variable'), extra = {'variable', 'level'}; end
    cols = [extra f];
    data = cell(numel(results), numel(cols));
    for i = 1:numel(results)
        for j = 1:numel(cols)
            data{i, j} = results(i).(cols{j});
        end
    end
    writeCsv(filename, cols, data);
end
