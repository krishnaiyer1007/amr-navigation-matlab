function ax = drawMap(ax, world, opts)
%DRAWMAP Draw known map, inflation and ground-truth blockers on axes AX.
%   opts.inflation (true)  show hard inflation used by the planner
%   opts.blockers  (true)  show hidden static blockers (ground truth, hatched red)
%   opts.title     ('')
    if nargin < 3, opts = struct(); end
    if ~isfield(opts, 'inflation'), opts.inflation = true; end
    if ~isfield(opts, 'blockers'), opts.blockers = true; end
    if ~isfield(opts, 'title'), opts.title = ''; end
    if ~isfield(opts, 'cfg'), opts.cfg = defaultConfig(); end
    hold(ax, 'on');
    img = ones(world.ny, world.nx, 3);
    if opts.inflation
        R = opts.cfg.robot.radius + opts.cfg.safety.margin + 0.5 / world.res;
        inf1 = inflateObstacles(world.known, R, world.res);
        img(repmat(inf1, [1 1 3])) = 0.86;
    end
    k = repmat(world.known, [1 1 3]);
    img(k) = 0.15;
    image(ax, [0.5 world.nx-0.5] / world.res, [0.5 world.ny-0.5] / world.res, img);
    set(ax, 'YDir', 'normal');
    if opts.blockers && isfield(world, 'blockers')
        for i = 1:numel(world.blockers)
            r = world.blockers(i).rect;
            patch(ax, r([1 3 3 1]), r([2 2 4 4]), [0.85 0.2 0.2], 'FaceAlpha', 0.6, ...
                  'EdgeColor', [0.6 0 0], 'LineWidth', 1);
        end
    end
    axis(ax, 'equal');
    xlim(ax, [0 world.width]); ylim(ax, [0 world.height]);
    xlabel(ax, 'x (m)'); ylabel(ax, 'y (m)');
    if ~isempty(opts.title), title(ax, opts.title, 'Interpreter', 'none'); end
    box(ax, 'on');
end
