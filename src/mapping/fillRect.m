function grid = fillRect(grid, rect, res)
%FILLRECT Mark cells whose centres lie inside rect = [x0 y0 x1 y1] as occupied.
    [ny, nx] = size(grid);
    c0 = max(1,  round(rect(1)*res) + 1);  c1 = min(nx, round(rect(3)*res));
    r0 = max(1,  round(rect(2)*res) + 1);  r1 = min(ny, round(rect(4)*res));
    if c1 >= c0 && r1 >= r0
        grid(r0:r1, c0:c1) = true;
    end
end
