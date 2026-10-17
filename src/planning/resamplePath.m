function q = resamplePath(p, spacing)
%RESAMPLEPATH Resample a polyline at (approximately) uniform arc length.
    if size(p,1) < 2, q = p; return; end
    d = [0; cumsum(sqrt(sum(diff(p).^2, 2)))];
    [d, ia] = unique(d, 'stable');
    p = p(ia, :);
    if numel(d) < 2, q = p(1,:); return; end
    L = d(end);
    n = max(2, ceil(L / spacing) + 1);
    s = linspace(0, L, n)';
    q = [interp1(d, p(:,1), s), interp1(d, p(:,2), s)];
end
