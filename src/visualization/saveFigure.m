function saveFigure(f, filename)
%SAVEFIGURE Export a figure to PNG (exportgraphics when available).
    try
        % exported figures always use the light theme (readable in reports,
        % independent of the MATLAB desktop theme, R2025a+)
        try, theme(f, 'light'); catch, end
        if exist('exportgraphics', 'file') || exist('exportgraphics', 'builtin')
            exportgraphics(f, filename, 'Resolution', 150);
        else
            print(f, filename, '-dpng', '-r150');
        end
    catch err
        warning('saveFigure:failed', 'Could not save %s: %s', filename, err.message);
    end
end
