function root = setupPaths()
%SETUPPATHS Add all project folders to the MATLAB path and return the root.
    here = fileparts(mfilename('fullpath'));        % src/utils
    root = fileparts(fileparts(here));              % project root
    folders = {'config', 'src', 'maps', 'scripts', 'tests', 'simulink'};
    for i = 1:numel(folders)
        p = fullfile(root, folders{i});
        if exist(p, 'dir'), addpath(genpath(p)); end
    end
    dirs = {fullfile(root,'results','figures'), fullfile(root,'results','logs'), ...
            fullfile(root,'results','tables')};
    for i = 1:numel(dirs)
        if ~exist(dirs{i}, 'dir'), mkdir(dirs{i}); end
    end
end
