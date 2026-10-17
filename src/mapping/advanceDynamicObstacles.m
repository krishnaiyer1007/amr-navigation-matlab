function world = advanceDynamicObstacles(world, t, dt)
%ADVANCEDYNAMICOBSTACLES Move scripted obstacles along their patrol routes.
%   Physically plausible motion: cruise speed d.speed, acceleration and
%   deceleration limited to world.obsAccel (m/s^2). Approaching the end of
%   the route the obstacle brakes to rest, dwells world.obsDwell seconds,
%   then accelerates back ('pingpong') or stays ('stop'). 'cycle' routes are
%   closed loops (no teleporting). Obstacles do not react to the robot.
    world = updateBlockers(world, t);
    a = world.obsAccel;
    for i = 1:numel(world.dyn)
        d = world.dyn(i);
        p0 = d.pos;
        if t < d.delay || d.speed <= 0 || d.done
            d.vel = [0 0];
            world.dyn(i) = d;
            continue;
        end
        if d.dwell > 0
            d.dwell = d.dwell - dt;
            d.vel = [0 0];
            world.dyn(i) = d;
            continue;
        end
        distEnd = distanceToEnd(d);
        vCap = sqrt(2 * a * distEnd);                       % brake to stop at the end
        d.curSpeed = min([d.curSpeed + a * dt, d.speed, vCap]);
        if distEnd > 1e-6
            d.curSpeed = max(d.curSpeed, min(0.05, distEnd / dt));   % finish the approach
        end
        remaining = d.curSpeed * dt;
        p = d.pos;
        guard = 0;
        while remaining > 1e-12 && guard < 20
            guard = guard + 1;
            if d.dir > 0, target = d.path(d.seg+1,:); else, target = d.path(d.seg,:); end
            v = target - p; L = norm(v);
            if L <= remaining
                p = target; remaining = remaining - L;
                n = size(d.path, 1);
                atEnd = (d.dir > 0 && d.seg + 1 >= n) || (d.dir < 0 && d.seg <= 1);
                if atEnd
                    d.curSpeed = 0; remaining = 0;
                    switch d.loop
                        case 'pingpong'
                            d.dir = -d.dir; d.dwell = world.obsDwell;
                        otherwise
                            d.done = true;
                    end
                elseif d.dir > 0
                    d.seg = d.seg + 1;
                else
                    d.seg = d.seg - 1;
                end
            else
                p = p + v / L * remaining; remaining = 0;
            end
        end
        d.vel = (p - p0) / dt;
        d.pos = p;
        world.dyn(i) = d;
    end
end

function L = distanceToEnd(d)
%DISTANCETOEND Remaining route length in the current direction.
    if strcmp(d.loop, 'cycle'), L = inf; return; end
    n = size(d.path, 1);
    if d.dir > 0
        pts = [d.pos; d.path(d.seg+1:n, :)];
    else
        pts = [d.pos; d.path(d.seg:-1:1, :)];
    end
    L = sum(sqrt(sum(diff(pts).^2, 2)));
end
