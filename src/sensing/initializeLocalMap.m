function lm = initializeLocalMap(world, cfg)
%INITIALIZELOCALMAP Local perception map for newly observed static obstacles.
%   lastSeen(r,c) = time the cell last produced a return that is NOT
%   explained by the prior map (-Inf = never / cleared). The known map is
%   dilated by staticTol so returns from known walls are not re-added.
    lm.lastSeen = -inf(world.ny, world.nx);
    lm.hits = zeros(world.ny, world.nx);
    lm.minHits = cfg.localMap.minHits;
    lm.res = world.res; lm.ny = world.ny; lm.nx = world.nx;
    tol = max(cfg.localMap.staticTol, cfg.localMap.noiseGate * cfg.sensor.rangeNoiseStd);
    k = ceil(tol * world.res);
    lm.knownDil = conv2(double(world.known), ones(2*k+1), 'same') > 0.5;
    lm.version = 0;           % increments when the set of occupied cells changes
    lm.occCount = 0;
end
