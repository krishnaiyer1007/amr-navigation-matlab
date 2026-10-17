function [cmd, stopped] = proximityStop(nominal, state, pts, tracks, cfg)
%PROXIMITYSTOP Simple reactive safety baseline (controller B1).
%   Stops forward motion when any return lies inside a forward corridor of
%   length radius + braking distance + b1StopDist, or when a tracked
%   obstacle's time-to-collision is below safety.ttcStop. Rotation is kept
%   so the robot can still turn; there is no replanning in B1.
    cmd = nominal; stopped = false;
    p = state.pose; R = cfg.robot.radius;
    v = max(state.v, nominal(1));
    stopDist = R + v^2 / (2 * cfg.robot.aMax) + cfg.safety.b1StopDist;
    if ~isempty(pts)
        c = cos(p(3)); s = sin(p(3));
        dx = pts(:,1) - p(1); dy = pts(:,2) - p(2);
        lon = c * dx + s * dy; lat = -s * dx + c * dy;
        if any(lon > 0 & lon < stopDist & abs(lat) < R + 0.05)
            stopped = true;
        end
    end
    L = tracks.list;
    for i = 1:numel(L)
        if ~L(i).moving, continue; end
        rel = L(i).pos - p(1:2);
        relV = L(i).vel - v * [cos(p(3)) sin(p(3))];
        dist = norm(rel) - L(i).r - R;
        closing = -dot(rel, relV) / max(norm(rel), 1e-6);
        if closing > 0 && dist / closing < cfg.safety.ttcStop
            stopped = true;
        end
    end
    if stopped, cmd(1) = 0; end
end
