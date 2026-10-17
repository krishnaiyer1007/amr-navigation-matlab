function isNew = classifyKnown(pts, lm)
%CLASSIFYKNOWN True for points NOT explained by the (dilated) known map.
    if isempty(pts), isNew = false(0,1); return; end
    [r, c] = world2grid(pts, lm.res, lm.ny, lm.nx);
    isNew = ~lm.knownDil(r + (c - 1) * lm.ny);
end
