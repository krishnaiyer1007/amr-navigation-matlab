function pts = knownBoundaryPoints(known, res)
%KNOWNBOUNDARYPOINTS Centres of occupied cells that touch free space.
%   A compact point representation of the prior map for local planning.
    k = [0 1 0; 1 0 1; 0 1 0];
    freeNb = conv2(double(~known), k, 'same') > 0;
    [r, c] = find(known & freeNb);
    pts = grid2world(r, c, res);
end
