function [cmd, info] = selectSafeVelocity(state, nominal, path, closestIdx, staticPts, dynObs, cfg)
%SELECTSAFEVELOCITY DWA-style sampled local planner with moving-obstacle prediction.
%   state     struct: pose [x y th] (estimated), v, w (current command)
%   nominal   [v w] from the tracking controller (added as a candidate)
%   path      reference path (Kx2), closestIdx = current progress index
%   staticPts Mx2 obstacle points treated as static (scan + maps)
%   dynObs    Kx5 [x y vx vy r] moving obstacle tracks (constant-velocity model)
%
%   Steps (README section 7.4):
%   1) candidate (v,w) reachable within reachTime under aMax/alphaMax
%   2) roll out each candidate: hold for commitTime, then brake at max
%      deceleration (stopping ability is part of the safety check)
%   3) moving obstacles predicted at constant velocity with an uncertainty
%      buffer unc0 + uncRate * tau
%   4) reject candidates whose footprint violates clearance at any step
%   5) score the rest: goal progress, path deviation, clearance,
%      smoothness, speed (all terms normalised to ~[0,1])
%   6) choose the lowest-cost safe FORWARD command; if none is safe ->
%      "evasive" command (sidestep or slow reverse down to robot.vMin)
%      that maximises the time to the first predicted violation of its
%      constant-command trajectory; if nothing is static-safe -> stop.
%   info.mode: 0 = normal, 1 = evasive, 2 = stop
    D = cfg.dwa; R = cfg.robot;
    rad = R.radius;
    v0 = state.v; w0 = state.w;
    T = D.reachTime;
    vs = linspace(max(0, v0 - R.aMax * T), min(R.vMax, v0 + R.aMax * T), D.nV);
    if R.vMin < 0
        % slow reverse candidates: only eligible in the evasive fallback
        vr = linspace(max(R.vMin, v0 - R.aMax * T), min(0, v0 + R.aMax * T), 3);
        vs = unique([vr(vr < 0) vs]);
    end
    ws = linspace(max(-R.wMax, w0 - R.alphaMax * T), min(R.wMax, w0 + R.alphaMax * T), D.nW);
    [VV, WW] = meshgrid(vs, ws);
    nomC = [min(max(nominal(1), vs(1)), vs(end)), min(max(nominal(2), ws(1)), ws(end))];
    brk = [max(0, v0 - R.aMax * T), sign(w0) * max(0, abs(w0) - R.alphaMax * T)];
    if v0 < 0, brk(1) = min(0, v0 + R.aMax * T); end
    V = [VV(:); nomC(1); brk(1)]; W = [WW(:); nomC(2); brk(2)];
    nc = numel(V);

    % ---- obstacle points near the robot ----
    p0 = state.pose;
    if ~isempty(staticPts)
        d2 = (staticPts(:,1) - p0(1)).^2 + (staticPts(:,2) - p0(2)).^2;
        keep = d2 < D.considerDist^2;
        S = staticPts(keep, :); d2 = d2(keep);
        if size(S,1) > D.maxPoints
            [~, o] = sort(d2); S = S(o(1:D.maxPoints), :);
        end
    else
        S = zeros(0, 2);
    end
    nd = size(dynObs, 1);

    % current static clearance (do not demand more than we have)
    if isempty(S), curClr = inf;
    else, curClr = sqrt(min((S(:,1) - p0(1)).^2 + (S(:,2) - p0(2)).^2)) - rad;
    end
    hardS = min(cfg.safety.hardMargin, max(0, curClr - 0.02));

    % ---- rollouts ----
    dt = D.simDt;
    nSteps = round(D.horizon / dt);
    nGoal = round(D.goalTime / dt);
    x = repmat(p0(1), nc, 1); y = repmat(p0(2), nc, 1); th = repmat(p0(3), nc, 1);
    xc = x; yc = y; thc = th;                       % constant-command rollout (scoring)
    vb = V; wb = W;                                 % braking rollout velocities
    minS = inf(nc, 1); minD = inf(nc, 1);           % braking: static / dynamic clearance
    tViol = inf(nc, 1);                             % constant command: first dynamic violation
    xe = x; ye = y; the = th;                       % constant-command rollout over full horizon
    minSe = inf(nc, 1);                             % its static clearance
    tStatE = inf(nc, 1);                            % its first static violation
    softClr = inf(nc, 1);                           % constant rollout clearance (to goalTime)
    for k = 1:nSteps
        tau = k * dt;
        if tau > D.commitTime
            vb = sign(vb) .* max(0, abs(vb) - R.aMax * dt);
            wb = sign(wb) .* max(0, abs(wb) - R.alphaMax * dt);
        end
        x = x + vb .* cos(th) * dt; y = y + vb .* sin(th) * dt; th = th + wb * dt;
        xe = xe + V .* cos(the) * dt; ye = ye + V .* sin(the) * dt; the = the + W * dt;
        if k == nGoal
            xc = xe; yc = ye; thc = the;
        end
        if ~isempty(S)
            ds = sqrt(min((x - S(:,1)').^2 + (y - S(:,2)').^2, [], 2)) - rad;
            minS = min(minS, ds);
            dse = sqrt(min((xe - S(:,1)').^2 + (ye - S(:,2)').^2, [], 2)) - rad;
            if k <= nGoal, softClr = min(softClr, dse); end
            newS = dse < hardS & isinf(tStatE);
            tStatE(newS) = tau;
        end
        if nd > 0
            ox = dynObs(:,1) + dynObs(:,3) * tau;
            oy = dynObs(:,2) + dynObs(:,4) * tau;
            buf = dynObs(:,5) + D.unc0 + D.uncRate * tau;
            dd = min(sqrt((x - ox').^2 + (y - oy').^2) - buf', [], 2) - rad;
            minD = min(minD, dd);
            dde = min(sqrt((xe - ox').^2 + (ye - oy').^2) - buf', [], 2) - rad;
            if k <= nGoal, softClr = min(softClr, dde); end
            newV = dde < cfg.safety.hardMargin & isinf(tViol);
            tViol(newV) = tau;
        end
    end
    okStatic = minS >= hardS;
    okDyn = minD >= cfg.safety.hardMargin;
    fwd = V >= 0;
    safe = okStatic & okDyn & fwd;            % normal operation: forward only

    % ---- cost ----
    [tgt, tgtIdx] = localTarget(path, closestIdx, D.targetDist, state.pose);
    scale = max(R.vMax * D.goalTime, 0.3);
    dGoal = sqrt((xc - tgt(1)).^2 + (yc - tgt(2)).^2) / scale;
    headErr = abs(wrapAngle(atan2(tgt(2) - yc, tgt(1) - xc) - thc)) / pi;
    if isempty(path)
        dPath = zeros(nc, 1);
    else
        lo = max(1, closestIdx); hi = min(size(path,1), max(tgtIdx, lo) + 10);
        P = path(lo:hi, :);
        dPath = sqrt(min((xc - P(:,1)').^2 + (yc - P(:,2)').^2, [], 2));
        dPath = min(dPath / 1.0, 1);
    end
    clrPen = max(0, D.clearSoft - min(softClr, D.clearSoft)) / D.clearSoft;
    smooth = ((V - state.v) / R.vMax).^2 + ((W - state.w) / R.wMax).^2;
    speed = (R.vMax - V) / R.vMax;
    J = D.wGoal * (dGoal + 0.3 * headErr) + D.wPath * dPath + D.wClear * clrPen + ...
        D.wSmooth * smooth + D.wSpeed * speed;

    info = struct('mode', 0, 'nSafe', nnz(safe), 'nCand', nc, 'target', tgt, ...
                  'minClr', 0);
    if any(safe)
        Jm = J; Jm(~safe) = inf;
        [~, b] = min(Jm);
    elseif any(okStatic)
        % no fully safe command: evade. Escape trajectories hold the command
        % for the whole horizon (sidestep / slow reverse) and are ranked by
        % the time until the first predicted violation (static or dynamic),
        % then by clearance and cost. Braking safety w.r.t. statics is kept.
        tEsc = min(tViol, tStatE);
        tEsc(~okStatic) = -inf;
        best = max(tEsc);
        cand = find(tEsc >= best - 1e-9);
        [~, j] = min(-minD(cand) + 0.05 * J(cand));
        b = cand(j);
        info.mode = 1;
    else
        cmd = brk; info.mode = 2; info.minClr = curClr;
        return;
    end
    cmd = [V(b), W(b)];
    info.minClr = min(minS(b), minD(b));
end

function [tgt, idx] = localTarget(path, ci, dist, pose)
    if isempty(path), tgt = pose(1:2); idx = 1; return; end
    n = size(path, 1);
    idx = min(max(ci, 1), n); acc = 0;
    while idx < n && acc < dist
        acc = acc + norm(path(idx+1,:) - path(idx,:)); idx = idx + 1;
    end
    tgt = path(idx, :);
end
