function sc = scenarioDefinitions(id, seed)
%SCENARIODEFINITIONS Repeatable test scenarios S01..S12.
%   sc = scenarioDefinitions(id)        nominal scenario (no randomisation)
%   sc = scenarioDefinitions(id, seed)  seeded randomised variant (seed > 0)
%   sc = scenarioDefinitions('list')    cell array of all scenario IDs
%
%   Fields of sc:
%     id, name, purpose, mapType, start [x y theta], goal [x y]
%     dyn       struct array: path (Nx2), speed (m/s), r (m), delay (s), loop
%     blockers  struct array: rect [x0 y0 x1 y1], tAppear (s)  (hidden statics)
%     overrides struct of cfg fields to change for this scenario
%     expect    'success' or 'unreachable'
%     seed      seed used for the variant
%   Moving obstacles are abstract circles (pedestrian / forklift stand-ins).

    if nargin < 2, seed = 0; end
    if ischar(id) && strcmpi(id, 'list')
        sc = arrayfun(@(k) sprintf('S%02d', k), 1:12, 'UniformOutput', false);
        return;
    end
    if isnumeric(id), id = sprintf('S%02d', id); end

    E = emptyDyn();
    B = emptyBlk();
    sc = struct('id', id, 'name', '', 'purpose', '', 'mapType', 'open', ...
                'start', [2 2 0], 'goal', [18 13], 'dyn', E, 'blockers', B, ...
                'overrides', struct(), 'expect', 'success', 'seed', seed);

    switch upper(id)
        case 'S01'
            sc.name = 'Empty open map';
            sc.purpose = 'Baseline kinematics and goal tracking';
            sc.mapType = 'open'; sc.start = [2 2 0]; sc.goal = [18 13];
        case 'S02'
            sc.name = 'Static shelf maze';
            sc.purpose = 'A* optimality and narrow corridors';
            sc.mapType = 'warehouse'; sc.start = [1.5 1.5 0]; sc.goal = [14 8];
        case 'S03'
            sc.name = 'Dead-end corridor';
            sc.purpose = 'Replanning and recovery';
            sc.mapType = 'warehouse'; sc.start = [1.5 1.5 0]; sc.goal = [18.5 13.5];
            sc.blockers = blk([17.0 10.3 19.8 10.7], 0);
        case 'S04'
            sc.name = 'Single moving obstacle crossing path';
            sc.purpose = 'Prediction and avoidance';
            sc.mapType = 'open'; sc.start = [2 7.5 0]; sc.goal = [18 7.5];
            sc.dyn = dyn([10 13.5; 10 1.5], 0.5, 0.30, 0, 'pingpong');
        case 'S05'
            sc.name = 'Two crossing obstacles';
            sc.purpose = 'Conflicting local constraints';
            sc.mapType = 'open'; sc.start = [2 7.5 0]; sc.goal = [18 7.5];
            sc.dyn = [dyn([7 11.5; 7 2], 0.5, 0.30, 0, 'pingpong'), ...
                      dyn([13 2; 13 13], 0.6, 0.35, 8, 'pingpong')];
        case 'S06'
            sc.name = 'Obstacle suddenly blocks corridor';
            sc.purpose = 'Blockage detection and replanning';
            sc.mapType = 'warehouse'; sc.start = [1.5 5 0]; sc.goal = [18.5 5];
            sc.blockers = blk([13 4 13.4 6], 6);
        case 'S07'
            sc.name = 'No feasible route';
            sc.purpose = 'Safe stop and correct failure status';
            sc.mapType = 'narrow'; sc.start = [3 7.5 0]; sc.goal = [17 7.5];
            sc.blockers = blk([9.8 6.9 10.2 8.1], 0);
            sc.expect = 'unreachable';
        case 'S08'
            sc.name = 'Narrow passage close to robot width';
            sc.purpose = 'Inflation and clearance';
            sc.mapType = 'narrow'; sc.start = [2 2 0]; sc.goal = [18 13];
        case 'S09'
            sc = scenarioDefinitions('S04', 0);
            sc.id = 'S09'; sc.seed = seed;
            sc.name = 'Noisy LiDAR and missed returns';
            sc.purpose = 'Perception robustness';
            sc.overrides.sensor = struct('rangeNoiseStd', 0.10, 'missProb', 0.15, ...
                                         'falseProb', 0.01);
        case 'S10'
            sc = scenarioDefinitions('S05', 0);
            sc.id = 'S10'; sc.seed = seed;
            sc.name = 'Higher robot speed';
            sc.purpose = 'Braking and stability margins';
            sc.overrides.robot = struct('vMax', 1.0);
        case 'S11'
            sc.name = 'Temporarily occluded moving obstacle';
            sc.purpose = 'Track expiry and uncertainty';
            sc.mapType = 'warehouse'; sc.start = [9.5 1.5 pi/2]; sc.goal = [9.5 13.6];
            sc.dyn = dyn([16 8; 4 8], 0.6, 0.30, 0, 'pingpong');
        case 'S12'
            sc.name = 'Multiple start/goal pairs';
            sc.purpose = 'Generalisation';
            sc.mapType = 'warehouse';
            % candidate start/goal points are kept >= 2 m from the patrol
            % routes: the scripted obstacles do not react to the robot, so a
            % goal on a patrol route would make collisions unavoidable.
            pts = [1.5 1.5; 1.5 8; 1.5 13.8; 9.5 1.5; 9.5 13.8; 18.5 1.5; ...
                   14 8; 18.5 13.8; 14 5; 14 11; 5.5 8; 5.5 11];
            k = mod(max(seed,1) * 7, size(pts,1)) + 1;
            j = mod(max(seed,1) * 5 + 3, size(pts,1)) + 1;
            if j == k, j = mod(j, size(pts,1)) + 1; end
            sc.start = [pts(k,:) 0]; sc.goal = pts(j,:);
            sc.start(3) = atan2(sc.goal(2)-sc.start(2), sc.goal(1)-sc.start(1));
            sc.dyn = [dyn([9.5 3.5; 9.5 11.8], 0.4, 0.30, 0, 'pingpong'), ...
                      dyn([4.0 1.5; 16 1.5],   0.5, 0.30, 0, 'pingpong'), ...
                      dyn([18.4 4.0; 18.4 11.8], 0.4, 0.30, 0, 'pingpong')];
        otherwise
            error('scenarioDefinitions:id', 'Unknown scenario %s', id);
    end

    % ---- seeded randomisation (repeatable) ----
    if seed > 0
        oldState = rng;                 % keep caller's generator untouched
        rng(seed, 'twister');
        for i = 1:numel(sc.dyn)
            sc.dyn(i).speed = sc.dyn(i).speed * (0.8 + 0.4 * rand());
            sc.dyn(i).delay = sc.dyn(i).delay + 3 * rand();
        end
        for i = 1:numel(sc.blockers)
            sc.blockers(i).tAppear = sc.blockers(i).tAppear * (0.8 + 0.4 * rand());
        end
        if ~strcmpi(sc.id, 'S12')
            sc.start(1:2) = sc.start(1:2) + 0.4 * (rand(1, 2) - 0.5);
            sc.goal       = sc.goal       + 0.4 * (rand(1, 2) - 0.5);
        end
        rng(oldState);
    end
end

function d = emptyDyn()
    d = struct('path', {}, 'speed', {}, 'r', {}, 'delay', {}, 'loop', {});
end

function b = emptyBlk()
    b = struct('rect', {}, 'tAppear', {});
end

function d = dyn(path, speed, r, delay, loop)
    d = struct('path', path, 'speed', speed, 'r', r, 'delay', delay, 'loop', loop);
end

function b = blk(rect, tAppear)
    b = struct('rect', rect, 'tAppear', tAppear);
end
