% testCollision  Footprint vs obstacle geometry: touching, overlap, just clear.
cfg = defaultConfig();
R = cfg.robot.radius;
world = createMap(cfg, 'open');
world.truth = world.known;
world.dyn = struct('pos', {}, 'r', {});

%% Robot in open space is collision free with capped clearance
[c, clr] = checkFootprintCollision([10 7.5 0], world, cfg);
assert(~c && abs(clr - 1.0) < 1e-9);

%% Overlapping a static cell is a collision
w = world; w.truth = fillRect(w.truth, [10 7 11 8], w.res);
[c, clr] = checkFootprintCollision([9.9 7.5 0], w, cfg);
assert(c && clr < 0, 'Overlap not detected');

%% Just clear of a wall (1 cm gap) is not a collision
w = world; w.truth = fillRect(w.truth, [10 7 11 8], w.res);
[c, clr] = checkFootprintCollision([10 - R - 0.01, 7.5, 0], w, cfg);
assert(~c && abs(clr - 0.01) < 1e-9, 'Clearance %.4f', clr);

%% Exactly touching is not counted as a collision (clearance 0)
w = world; w.truth = fillRect(w.truth, [10 7 11 8], w.res);
[c, clr] = checkFootprintCollision([10 - R, 7.5, 0], w, cfg);
assert(~c && abs(clr) < 1e-9);

%% Corner distance uses the cell square, not its centre
w = world; w.truth = fillRect(w.truth, [10 7 11 8], w.res);
[~, clr] = checkFootprintCollision([10 - 0.3, 7 - 0.4, 0], w, cfg);
assert(abs(clr - (0.5 - R)) < 1e-9, 'Corner distance wrong: %.4f', clr);

%% Dynamic circle: touching, overlapping and clear
w = world; w.dyn = struct('pos', [12 7.5], 'r', 0.3);
[c, clr] = checkFootprintCollision([12 - 0.3 - R, 7.5, 0], w, cfg);
assert(~c && abs(clr) < 1e-9);
c = checkFootprintCollision([12 - 0.3 - R + 0.01, 7.5, 0], w, cfg);
assert(c, 'Dynamic overlap not detected');
[c, clr] = checkFootprintCollision([12 - 0.3 - R - 0.05, 7.5, 0], w, cfg);
assert(~c && abs(clr - 0.05) < 1e-9);
