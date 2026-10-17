function out = astarGrid(occ, startXY, goalXY, res, opts)
%ASTARGRID A* search on an 8-connected occupancy grid.
%   out = astarGrid(occ, startXY, goalXY, res) plans on logical grid OCC
%   (true = blocked; normally an INFLATED grid) from world point startXY
%   to goalXY (metres).
%
%   - Edge cost: 1 (axial) or sqrt(2) (diagonal) cells, scaled by 1/res.
%   - Heuristic: octile distance (admissible and consistent).
%   - Diagonal corner cutting is forbidden: a diagonal move is illegal if
%     either adjacent axial cell is blocked.
%   - Binary heap open list with lazy deletion.
%
%   opts.costMap (optional, ny x nx, >= 0): extra traversal cost per cell
%   (in cells of distance per cell entered, scaled by move length). Used to
%   keep paths away from obstacles; the octile heuristic stays admissible.
%
%   opts.snapStart (default true): if the start cell is blocked (robot is
%   inside the inflation band), start from the nearest free cell within
%   opts.snapRadius metres (default 0.6) - typical after close passes.
%
%   Output struct:
%     status    'success' | 'start_blocked' | 'goal_blocked' | 'no_path' | 'out_of_bounds'
%     success   logical
%     path      Kx2 waypoints in world metres (cell centres; first = start point)
%     cells     Kx2 [row col]
%     length    path length (m)
%     expanded  number of expanded nodes
    if nargin < 5, opts = struct(); end
    if ~isfield(opts, 'snapStart'), opts.snapStart = true; end
    if ~isfield(opts, 'snapRadius'), opts.snapRadius = 0.6; end
    if ~isfield(opts, 'costMap') || isempty(opts.costMap)
        extra = zeros(numel(occ), 1);
    else
        extra = opts.costMap(:);
    end

    [ny, nx] = size(occ);
    out = struct('status', '', 'success', false, 'path', zeros(0,2), ...
                 'cells', zeros(0,2), 'length', Inf, 'expanded', 0, 'cost', Inf);

    [sr, sc, sIn] = world2grid(startXY(1:2), res, ny, nx);
    [gr, gc, gIn] = world2grid(goalXY(1:2), res, ny, nx);
    if ~sIn || ~gIn
        out.status = 'out_of_bounds'; return;
    end
    if occ(gr, gc)
        out.status = 'goal_blocked'; return;
    end
    if occ(sr, sc)
        if opts.snapStart
            [sr, sc, ok] = nearestFree(occ, sr, sc, ceil(opts.snapRadius * res));
            if ~ok, out.status = 'start_blocked'; return; end
        else
            out.status = 'start_blocked'; return;
        end
    end

    N = ny * nx;
    g = inf(N, 1);
    parent = zeros(N, 1, 'uint32');
    closed = false(N, 1);
    blocked = occ(:);

    sIdx = sub2ind([ny nx], sr, sc);
    gIdx = sub2ind([ny nx], gr, gc);
    g(sIdx) = 0;

    % neighbour offsets: [dr dc cost]
    nb = [ 1 0 1; -1 0 1; 0 1 1; 0 -1 1; ...
           1 1 sqrt(2); 1 -1 sqrt(2); -1 1 sqrt(2); -1 -1 sqrt(2)];

    % binary min-heap of [f, idx]
    cap = 4096; heapF = zeros(cap,1); heapI = zeros(cap,1); hn = 0;
    [heapF, heapI, hn] = heapPush(heapF, heapI, hn, octile(sr, sc, gr, gc), sIdx);

    expanded = 0;
    found = false;
    while hn > 0
        [f, cur, heapF, heapI, hn] = heapPop(heapF, heapI, hn); %#ok<ASGLU>
        if closed(cur), continue; end
        closed(cur) = true;
        expanded = expanded + 1;
        if cur == gIdx, found = true; break; end
        cr = mod(cur - 1, ny) + 1;
        cc = floor((cur - 1) / ny) + 1;
        for k = 1:8
            r2 = cr + nb(k,1); c2 = cc + nb(k,2);
            if r2 < 1 || r2 > ny || c2 < 1 || c2 > nx, continue; end
            n2 = r2 + (c2 - 1) * ny;
            if blocked(n2) || closed(n2), continue; end
            if k > 4
                % corner-cut prevention
                if blocked(cr + nb(k,1) + (cc - 1) * ny) || blocked(cr + (c2 - 1) * ny)
                    continue;
                end
            end
            ng = g(cur) + nb(k,3) * (1 + extra(n2));
            if ng < g(n2)
                g(n2) = ng;
                parent(n2) = cur;
                if hn >= numel(heapF)
                    heapF = [heapF; zeros(numel(heapF),1)]; %#ok<AGROW>
                    heapI = [heapI; zeros(numel(heapI),1)]; %#ok<AGROW>
                end
                [heapF, heapI, hn] = heapPush(heapF, heapI, hn, ng + octile(r2, c2, gr, gc), n2);
            end
        end
    end
    out.expanded = expanded;
    if ~found
        out.status = 'no_path'; return;
    end

    % reconstruct
    idx = zeros(expanded + 1, 1); n = 0; cur = gIdx;
    while cur ~= 0
        n = n + 1; idx(n) = cur;
        if cur == sIdx, break; end
        cur = double(parent(cur));
    end
    idx = flipud(idx(1:n));
    [rr, cc] = ind2sub([ny nx], idx);
    out.cells = [rr cc];
    pts = grid2world(rr, cc, res);
    pts(1,:) = startXY(1:2);           % exact start
    pts(end,:) = goalXY(1:2);          % exact goal
    out.path = pts;
    out.length = sum(sqrt(sum(diff(pts).^2, 2)));
    out.cost = g(gIdx) / res;
    out.status = 'success';
    out.success = true;
end

function h = octile(r, c, gr, gc)
    dr = abs(r - gr); dc = abs(c - gc);
    h = max(dr, dc) + (sqrt(2) - 1) * min(dr, dc);
end

function [hf, hi, n] = heapPush(hf, hi, n, f, idx)
    n = n + 1; k = n;
    while k > 1
        p = floor(k / 2);
        if hf(p) <= f, break; end
        hf(k) = hf(p); hi(k) = hi(p); k = p;
    end
    hf(k) = f; hi(k) = idx;
end

function [f, idx, hf, hi, n] = heapPop(hf, hi, n)
    f = hf(1); idx = hi(1);
    lf = hf(n); li = hi(n); n = n - 1;
    k = 1;
    while true
        c = 2 * k;
        if c > n, break; end
        if c < n && hf(c+1) < hf(c), c = c + 1; end
        if hf(c) >= lf, break; end
        hf(k) = hf(c); hi(k) = hi(c); k = c;
    end
    if n >= 1
        hf(k) = lf; hi(k) = li;
    end
end

function [r, c, ok] = nearestFree(occ, r0, c0, maxK)
    [ny, nx] = size(occ);
    ok = false; r = r0; c = c0; best = inf;
    for k = 1:maxK
        for dr = -k:k
            for dc = -k:k
                if max(abs(dr), abs(dc)) ~= k, continue; end
                rr = r0 + dr; cc = c0 + dc;
                if rr < 1 || rr > ny || cc < 1 || cc > nx || occ(rr, cc), continue; end
                d = dr^2 + dc^2;
                if d < best, best = d; r = rr; c = cc; ok = true; end
            end
        end
        if ok, return; end
    end
end
