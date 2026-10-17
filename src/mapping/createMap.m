function world = createMap(cfg, mapType)
%CREATEMAP Build the KNOWN static occupancy grid for a named layout.
%   world = createMap(cfg, mapType) returns a struct with fields
%     known      logical ny x nx occupancy grid (1 = occupied) known a priori
%     res, nx, ny, width, height, mapType, rects (list of known rectangles)
%   mapType: 'open' | 'warehouse' | 'narrow' | 'walled'
%   Grid convention: grid(row, col); row 1 = bottom (y=0). See world2grid.

    if nargin < 2, mapType = 'warehouse'; end
    res = cfg.map.resolution;
    W = cfg.map.width; H = cfg.map.height;
    nx = round(W * res); ny = round(H * res);
    t = 0.2;                                  % wall thickness, m

    rects = [0 0 W t; 0 H-t W H; 0 0 t H; W-t 0 W H];   % [x0 y0 x1 y1] boundary

    switch lower(mapType)
        case 'open'
            % boundary only
        case 'warehouse'
            % Two shelf blocks with 4 shelf rows each. Aisles run along x,
            % a 3 m cross-aisle at x = 8..11 and side corridors at both ends.
            ys = [3 4; 6 7; 9 10; 12 12.8];
            for i = 1:size(ys,1)
                rects(end+1,:) = [3  ys(i,1) 8  ys(i,2)]; %#ok<AGROW>
                rects(end+1,:) = [11 ys(i,1) 17 ys(i,2)]; %#ok<AGROW>
            end
        case 'narrow'
            % Dividing wall at x = 9.8..10.2 with a single 1.0 m doorway
            % centred at y = 7.5 (robot diameter 0.44 m).
            rects(end+1,:) = [9.8 t   10.2 7.0];
            rects(end+1,:) = [9.8 8.0 10.2 H-t];
            % a few boxes to make the approach non-trivial
            rects(end+1,:) = [4 3 6 5];
            rects(end+1,:) = [4 10 6 12];
            rects(end+1,:) = [14 5 16 6];
        case 'walled'
            % Same dividing wall as 'narrow' but with a 1.6 m doorway
            rects(end+1,:) = [9.8 t   10.2 6.7];
            rects(end+1,:) = [9.8 8.3 10.2 H-t];
        otherwise
            error('createMap:type', 'Unknown map type "%s"', mapType);
    end

    known = false(ny, nx);
    for i = 1:size(rects,1)
        known = fillRect(known, rects(i,:), res);
    end

    world.known   = known;
    world.res     = res;
    world.nx      = nx;
    world.ny      = ny;
    world.width   = W;
    world.height  = H;
    world.mapType = mapType;
    world.rects   = rects;
end
