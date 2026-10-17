function inflated = inflateObstacles(grid, radius, res)
%INFLATEOBSTACLES Grow occupied cells by RADIUS metres (disk structuring element).
%   A cell is marked occupied when its centre is within RADIUS of the
%   centre of any occupied cell. Uses conv2 (core MATLAB, no toolbox).
%   For path planning use radius = robot.radius + safety.margin.
    k = floor(radius * res + 1e-9);
    if k <= 0
        inflated = logical(grid);
        return;
    end
    [dx, dy] = meshgrid(-k:k, -k:k);
    disk = double((dx.^2 + dy.^2) <= (radius * res)^2 + 1e-9);
    inflated = conv2(double(grid), disk, 'same') > 0.5;
end
