% testMapping  Ray casting and local map behaviour (noise-free geometry).
cfg = defaultConfig();
cfg.sensor.rangeNoiseStd = 0; cfg.sensor.missProb = 0; cfg.sensor.falseProb = 0;
world = createMap(cfg, 'open');
world.truth = fillRect(world.known, [5 0.2 5.5 14.8], world.res);   % wall at x = 5
world.dyn = struct('pos', {}, 'r', {});
mid = (cfg.sensor.numBeams + 1) / 2;

%% Forward beam hits a known wall at the right range
scan = simulateRangeScan([2 7.5 0], world, cfg);
assert(abs(scan.angles(mid)) < 1e-12);
assert(abs(scan.ranges(mid) - 3.0) <= 0.05 + 1e-9, 'Range %.3f', scan.ranges(mid));

%% Oblique beams follow 1/cos geometry
scan = simulateRangeScan([2 7.5 0], world, cfg);
for k = mid + (1:10)
    a = scan.angles(k);
    if isnan(scan.ranges(k)), continue; end
    assert(abs(scan.ranges(k) - 3.0 / cos(a)) < 0.06, 'Beam %d', k);
end

%% Max range and field of view
scan = simulateRangeScan([12 7.5 0], world, cfg);        % facing +x, 7.8 m to wall
assert(all(isnan(scan.ranges(mid-5:mid+5))), 'Beyond max range must be no-return');
assert(abs(scan.angles(1) + 135*pi/180) < 1e-12 && abs(scan.angles(end) - 135*pi/180) < 1e-12);

%% Dynamic circle intersection is analytic
w = world; w.truth = world.known;
w.dyn = struct('pos', [6 7.5], 'r', 0.5);
scan = simulateRangeScan([2 7.5 0], w, cfg);
assert(abs(scan.ranges(mid) - 3.5) < 1e-9);

%% Noise statistics match configuration
c2 = cfg; c2.sensor.rangeNoiseStd = 0.05;
rng(1);
e = zeros(1, 200);
for i = 1:200
    s = simulateRangeScan([2 7.5 0], world, c2);
    e(i) = s.ranges(mid) - s.trueRanges(mid);
end
assert(abs(std(e) - 0.05) < 0.01 && abs(mean(e)) < 0.02);

%% Missed returns occur at the configured rate
c3 = cfg; c3.sensor.missProb = 0.2; rng(2);
s = simulateRangeScan([2 7.5 0], world, c3);
hitBeams = isfinite(s.trueRanges);
rate = 1 - nnz(s.valid & hitBeams) / nnz(hitBeams);
assert(rate > 0.08 && rate < 0.35, 'Miss rate %.2f', rate);

%% Local map stamps new obstacles, ignores known walls, clears free space
w = world; w.known = fillRect(world.known, [7.5 0.2 8 14.8], w.res);  % known back wall
w.truth = fillRect(w.known, [5 6 5.5 9], w.res);                       % unknown obstacle
lm = initializeLocalMap(w, cfg);
tr = initializeTracks();
for k = 1:3
    s = simulateRangeScan([2 7.5 0], w, cfg);
    [pts, bi] = scanToPoints(s, [2 7.5 0]);
    isNew = classifyKnown(pts, lm);
    [tr, mv, ~, inCl] = estimateObstacleMotion(tr, pts, bi, isNew, [2 7.5], k*0.05, cfg);
    lm = updateLocalMap(lm, s, [2 7.5 0], pts, bi, inCl, mv, k*0.05, cfg);
end
[occ, P] = observedGrid(lm);
n0 = nnz(occ);
assert(n0 > 5 && all(abs(P(:,1) - 5.05) < 0.11), 'Unknown obstacle not mapped at x = 5');
assert(~any(any(occ(:, 1:3))), 'Boundary walls (known) were re-added');
% remove obstacle -> rays to the known back wall clear the cells
w.truth = w.known;
for k = 4:6
    s = simulateRangeScan([2 7.5 0], w, cfg);
    [pts, bi] = scanToPoints(s, [2 7.5 0]);
    isNew = classifyKnown(pts, lm);
    [tr, mv, ~, inCl] = estimateObstacleMotion(tr, pts, bi, isNew, [2 7.5], k*0.05, cfg);
    lm = updateLocalMap(lm, s, [2 7.5 0], pts, bi, inCl, mv, k*0.05, cfg);
end
occ = observedGrid(lm);
assert(nnz(occ) <= 0.2 * n0, 'Free-space clearing failed (%d of %d left)', nnz(occ), n0);
% cells grazed by no beam expire after localMap.memory seconds
tLate = 6 * 0.05 + cfg.localMap.memory + 0.1;
lm = updateLocalMap(lm, s, [2 7.5 0], zeros(0,2), zeros(0,1), false(0,1), false(0,1), tLate, cfg);
assert(nnz(observedGrid(lm)) == 0, 'Stale cells did not expire');

%% Tracker estimates the velocity of a moving obstacle
w = world; w.truth = world.known;
w.dyn = struct('pos', [6 4], 'r', 0.3, 'vel', [0 0.5]);
lm = initializeLocalMap(w, cfg); tr = initializeTracks();
for k = 1:60
    w.dyn.pos = [6, 4 + 0.5 * k * cfg.dt];
    s = simulateRangeScan([2 7.5 0], w, cfg);
    [pts, bi] = scanToPoints(s, [2 7.5 0]);
    isNew = classifyKnown(pts, lm);
    tr = estimateObstacleMotion(tr, pts, bi, isNew, [2 7.5], k * cfg.dt, cfg);
end
assert(numel(tr.list) == 1, 'Expected one track, got %d', numel(tr.list));
assert(tr.list(1).moving && norm(tr.list(1).vel - [0 0.5]) < 0.12, ...
       'Velocity estimate [%.2f %.2f]', tr.list(1).vel);
