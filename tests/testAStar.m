% testAStar  Grid A* correctness: optimality, corner cutting, failures.
res = 1;   % 1 cell = 1 m for easy reasoning

%% Shortest path on an empty grid uses octile distance
occ = false(10, 10);
o = astarGrid(occ, [0.5 0.5], [9.5 4.5], res);
assert(o.success, 'Should succeed');
expected = 4 * sqrt(2) + 5;              % 4 diagonal + 5 straight moves
assert(abs(o.length - expected) < 1e-9, 'Path not optimal: %.3f vs %.3f', o.length, expected);

%% Path around a wall matches brute-force Dijkstra
occ = false(12, 12); occ(2:11, 6) = true;
o = astarGrid(occ, [1.5 1.5], [10.5 1.5], res);
assert(o.success);
dj = dijkstraRef(occ, [2 2], [2 11]);
assert(abs(o.length - dj) < 1e-9, 'A* length %.4f differs from Dijkstra %.4f', o.length, dj);

%% No diagonal corner cutting
occ = false(3, 3); occ(1, 2) = true; occ(2, 1) = true;   % only a diagonal gap
o = astarGrid(occ, [0.5 0.5], [1.5 1.5], res, struct('snapStart', false));
assert(~o.success && strcmp(o.status, 'no_path'), 'Corner cut through diagonal gap');

%% Returned path never cuts a blocked corner
occ = false(8, 8); occ(4, 3:6) = true;
o = astarGrid(occ, [3.5 1.5], [3.5 6.5], res);
assert(o.success);
for k = 1:size(o.cells,1)-1
    a = o.cells(k,:); b = o.cells(k+1,:);
    if all(abs(b - a) == 1)
        assert(~occ(a(1), b(2)) && ~occ(b(1), a(2)), 'Corner cut at step %d', k);
    end
end

%% Failure when no path exists
occ = false(10, 10); occ(:, 5) = true;
o = astarGrid(occ, [1.5 1.5], [8.5 8.5], res);
assert(~o.success && strcmp(o.status, 'no_path'), 'Expected no_path');

%% Goal inside an obstacle is rejected
occ = false(10, 10); occ(9, 9) = true;
o = astarGrid(occ, [1.5 1.5], [8.5 8.5], res);
assert(strcmp(o.status, 'goal_blocked'), 'Expected goal_blocked');

%% Start inside the inflation band snaps to the nearest free cell
occ = false(10, 10); occ(2, 2) = true;
o = astarGrid(occ, [1.5 1.5], [8.5 8.5], res);
assert(o.success, 'Start snapping failed');
o = astarGrid(occ, [1.5 1.5], [8.5 8.5], res, struct('snapStart', false));
assert(strcmp(o.status, 'start_blocked'));

%% Inflation radius on the real map and world/grid conversion
cfg = defaultConfig(); w = createMap(cfg, 'warehouse');
infl = inflateObstacles(w.known, 0.37, w.res);
assert(nnz(infl) > nnz(w.known));
[r, c] = world2grid([1.55 2.25], w.res, w.ny, w.nx);
assert(r == 23 && c == 16, 'world2grid wrong');
xy = grid2world(r, c, w.res);
assert(norm(xy - [1.55 2.25]) < 1e-9, 'grid2world wrong');

%% Warehouse plan is valid and smoothed path stays collision free
cfg = defaultConfig(); w = createMap(cfg, 'warehouse');
[o, p, pg] = planGlobalPath(w.known, [1.5 1.5], [18.5 13.5], cfg);
assert(o.success && size(p, 1) > 10);
[rr, cc] = world2grid(p, w.res, w.ny, w.nx);
assert(~any(pg(sub2ind(size(pg), rr, cc))), 'Smoothed path enters inflated space');
