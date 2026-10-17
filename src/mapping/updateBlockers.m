function world = updateBlockers(world, t)
%UPDATEBLOCKERS Activate hidden static blockers whose appearance time passed.
    changed = false;
    for i = 1:numel(world.blockers)
        if ~world.blockers(i).active && t >= world.blockers(i).tAppear
            world.blockers(i).active = true;
            changed = true;
        end
    end
    if changed
        world.truth = world.known;
        for i = 1:numel(world.blockers)
            if world.blockers(i).active
                world.truth = fillRect(world.truth, world.blockers(i).rect, world.res);
            end
        end
    end
end
