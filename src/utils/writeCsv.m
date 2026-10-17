function writeCsv(filename, header, data)
%WRITECSV Write a cell array (rows x cols) with a header row to CSV.
%   Works in core MATLAB and GNU Octave (no table dependency).
    fid = fopen(filename, 'w');
    if fid < 0, error('writeCsv:open', 'Cannot open %s', filename); end
    cleaner = onCleanup(@() fclose(fid));
    fprintf(fid, '%s\n', strjoin(header, ','));
    for i = 1:size(data, 1)
        parts = cell(1, size(data, 2));
        for j = 1:size(data, 2)
            v = data{i, j};
            if ischar(v)
                parts{j} = v;
            elseif islogical(v)
                parts{j} = sprintf('%d', v);
            elseif isempty(v) || (isnumeric(v) && isnan(v))
                parts{j} = 'NaN';
            elseif isnumeric(v) && v == round(v) && abs(v) < 1e9
                parts{j} = sprintf('%d', v);
            else
                parts{j} = sprintf('%.6g', v);
            end
        end
        fprintf(fid, '%s\n', strjoin(parts, ','));
    end
end
