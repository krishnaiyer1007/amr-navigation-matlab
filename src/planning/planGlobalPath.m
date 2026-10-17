function [out, path, planGrid] = planGlobalPath(occBase, startXY, goalXY, cfg)
%PLANGLOBALPATH Inflate, plan with A* (with obstacle-proximity cost), smooth.
%   occBase  raw occupancy (prior map OR prior map + observed obstacles)
%   1) hard inflation by robot.radius + safety.margin (+ half a cell to
%      account for cell extent) -> planGrid
%   2) soft cost band of planner.costBand metres beyond the hard inflation
%      with weight planner.costWeight -> paths prefer aisle centres
%   3) A* (8-connected, octile, no corner cutting)
%   4) line-of-sight smoothing on the band grid where possible, then
%      resampling at planner.waypointSpacing
    res = cfg.map.resolution;
    Rinf = cfg.robot.radius + cfg.safety.margin + 0.5 / res;
    planGrid = inflateObstacles(occBase, Rinf, res);
    band = inflateObstacles(occBase, Rinf + cfg.planner.costBand, res) & ~planGrid;
    costMap = cfg.planner.costWeight * double(band);
    out = astarGrid(planGrid, startXY, goalXY, res, struct('costMap', costMap));
    path = zeros(0, 2);
    if out.success
        if cfg.planner.smooth
            path = smoothPath(out.path, planGrid | band, res, cfg.planner.waypointSpacing);
        else
            path = resamplePath(out.path, cfg.planner.waypointSpacing);
        end
    end
end
