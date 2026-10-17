% testReplanning  Blockage detection, valid alternative route, safe stop.
cfg = defaultConfig();
world = createMap(cfg, 'warehouse');

%% New obstacle on the path is detected as a blockage
[o, path] = planGlobalPath(world.known, [1.5 5], [18.5 5], cfg);
assert(o.success);
blk = fillRect(false(size(world.known)), [13 4 13.4 6], world.res);
[r, c] = find(blk); obsPts = grid2world(r, c, world.res);
R = cfg.robot.radius + 0.5 * cfg.safety.margin;
assert(isPathBlocked(path, 1, obsPts, R, 100), 'Blockage not detected');
assert(~isPathBlocked(path, 1, obsPts, R, 3), 'Blockage beyond check distance reported');

%% Replanning on the merged map gives a valid alternative
[o, path] = planGlobalPath(world.known, [1.5 5], [18.5 5], cfg);
blk = fillRect(false(size(world.known)), [13 4 13.4 6], world.res);
[r, c] = find(blk); obsPts = grid2world(r, c, world.res);
R = cfg.robot.radius + 0.5 * cfg.safety.margin;
[o2, p2, pg] = planGlobalPath(world.known | blk, [1.5 5], [18.5 5], cfg);
assert(o2.success, 'No alternative route found');
assert(~isPathBlocked(p2, 1, obsPts, R, 100), 'New path still blocked');
[rr, cc] = world2grid(p2, world.res, world.ny, world.nx);
assert(~any(pg(sub2ind(size(pg), rr, cc))), 'New path enters inflated space');
assert(o2.length > o.length, 'Detour should be longer');

%% No feasible route returns a clear failure status
w = createMap(cfg, 'narrow');
plug = fillRect(w.known, [9.8 6.9 10.2 8.1], w.res);
o3 = planGlobalPath(plug, [3 7.5], [17 7.5], cfg);
assert(~o3.success && strcmp(o3.status, 'no_path'));

%% Closed loop: sudden blockage (S06) is replanned around and reached
[res, lg] = runSimulation(scenarioDefinitions('S06'), 'P', cfg);
assert(res.success, 'S06 failed: %s', res.reason);
assert(res.replans >= 1 && res.collisions == 0);
assert(any(lg.blocked), 'Blockage never flagged');

%% Closed loop: no feasible route (S07) ends in a safe stop
[res, lg] = runSimulation(scenarioDefinitions('S07'), 'P', cfg);
assert(strcmp(res.reason, 'unreachable'), 'Got %s', res.reason);
assert(res.collisions == 0 && abs(lg.vel(end,1)) < 0.02, 'Robot not stopped');
