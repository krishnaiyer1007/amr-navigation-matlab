function e = simEvent(t, type, detail, pose)
%SIMEVENT Event record for the run log.
    e = struct('t', t, 'type', type, 'detail', detail, 'x', pose(1), 'y', pose(2));
end
