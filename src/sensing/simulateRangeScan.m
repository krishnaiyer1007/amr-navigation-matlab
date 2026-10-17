function scan = simulateRangeScan(pose, world, cfg)
%SIMULATERANGESCAN Simulated 2D LiDAR by ray casting against GROUND TRUTH.
%   Static occupancy (world.truth, including hidden blockers) is ray
%   marched at half-cell steps; dynamic obstacles (circles) are intersected
%   analytically. Adds Gaussian range noise, missed returns (missProb) and
%   spurious short returns (falseProb). Random draws are made for every
%   beam on every call so the random stream stays aligned across
%   controllers (fair paired comparisons).
%
%   scan.angles  1xB beam angles in the body frame (rad)
%   scan.ranges  1xB measured ranges (NaN = no return)
%   scan.valid   1xB logical
%   scan.trueRanges 1xB noise-free ranges (Inf = nothing within maxRange)
    S = cfg.sensor;
    B = S.numBeams;
    ang = linspace(-S.fov/2, S.fov/2, B) * pi / 180;
    a = pose(3) + ang;
    ux = cos(a); uy = sin(a);

    % --- static: ray march ---
    step = 0.5 / world.res;
    s = (step:step:S.maxRange)';
    X = pose(1) + s * ux;  Y = pose(2) + s * uy;      % ns x B
    c = floor(X * world.res) + 1; r = floor(Y * world.res) + 1;
    out = r < 1 | r > world.ny | c < 1 | c > world.nx;
    r = min(max(r, 1), world.ny); c = min(max(c, 1), world.nx);
    hit = world.truth(r + (c - 1) * world.ny) | out;
    [anyHit, first] = max(hit, [], 1);
    rng0 = inf(1, B);
    rng0(anyHit) = s(first(anyHit))';

    % --- dynamic circles: analytic intersection ---
    for i = 1:numel(world.dyn)
        d = [pose(1) - world.dyn(i).pos(1), pose(2) - world.dyn(i).pos(2)];
        bq = ux * d(1) + uy * d(2);
        cq = d(1)^2 + d(2)^2 - world.dyn(i).r^2;
        disc = bq.^2 - cq;
        tHit = -bq - sqrt(max(disc, 0));
        ok = disc >= 0 & tHit > 0;
        rng0(ok) = min(rng0(ok), tHit(ok));
    end
    rng0(rng0 > S.maxRange) = inf;

    % --- noise model (fixed number of random draws per call) ---
    nz = randn(1, B) * S.rangeNoiseStd;
    um = rand(1, B);
    uf = rand(1, B);
    uv = rand(1, B);
    meas = rng0 + nz;
    valid = isfinite(rng0) & um >= S.missProb;
    fakes = uf < S.falseProb;
    meas(fakes) = 0.3 + uv(fakes) * (S.maxRange - 0.3);
    valid = valid | fakes;
    meas(~valid) = NaN;
    meas = min(max(meas, 0.05), S.maxRange);
    meas(~valid) = NaN;

    scan.angles = ang;
    scan.ranges = meas;
    scan.valid = valid;
    scan.trueRanges = rng0;
end
