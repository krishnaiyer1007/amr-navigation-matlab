function world = initWorld(cfg, sc)
%INITWORLD Ground-truth world state for one scenario run.
%   Adds hidden static obstacles (blockers), dynamic obstacles and the
%   truth occupancy grid to the known map produced by createMap.
%   The navigation stack must only see world.known (prior map) and sensor
%   data; world.truth and world.dyn are GROUND TRUTH used by the simulator.

    world = createMap(cfg, sc.mapType);
    world.start = sc.start(:)';
    world.goal  = sc.goal(:)';
    world.truth = world.known;

    % hidden static blockers, possibly appearing later
    world.blockers = sc.blockers;
    for i = 1:numel(world.blockers)
        world.blockers(i).active = false;
    end
    world = updateBlockers(world, 0);

    % dynamic obstacles: piecewise-linear patrol paths
    nd = numel(sc.dyn);
    world.obsAccel = cfg.obstacles.accel;     % m/s^2, obstacle speed changes
    world.obsDwell = cfg.obstacles.dwell;     % s, pause at patrol ends
    world.dyn = struct('pos', cell(1,nd), 'vel', [], 'r', [], 'path', [], ...
                       'speed', [], 'seg', [], 'dir', [], 'delay', [], 'loop', [], ...
                       'curSpeed', [], 'dwell', [], 'done', []);
    for i = 1:nd
        d = sc.dyn(i);
        if strcmp(d.loop, 'cycle') && norm(d.path(end,:) - d.path(1,:)) > 1e-9
            d.path(end+1,:) = d.path(1,:);         % closed loop, no teleport
        end
        world.dyn(i).path  = d.path;
        world.dyn(i).pos   = d.path(1,:);
        world.dyn(i).vel   = [0 0];
        world.dyn(i).r     = d.r;
        world.dyn(i).speed = d.speed;
        world.dyn(i).seg   = 1;
        world.dyn(i).dir   = 1;
        world.dyn(i).delay = d.delay;
        world.dyn(i).loop  = d.loop;
        world.dyn(i).curSpeed = d.speed * (d.delay <= 0);   % already moving unless delayed
        world.dyn(i).dwell = 0;
        world.dyn(i).done  = false;
    end
end
