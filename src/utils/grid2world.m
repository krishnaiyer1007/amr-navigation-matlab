function xy = grid2world(r, c, res)
%GRID2WORLD Convert grid (row, col) to world coordinates of the cell centre.
    xy = [(c(:) - 0.5) / res, (r(:) - 0.5) / res];
end
