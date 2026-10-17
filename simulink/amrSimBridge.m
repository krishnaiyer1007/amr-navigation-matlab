function varargout = amrSimBridge(action, varargin)
%AMRSIMBRIDGE Link between the Simulink model and the shared navigation code.
%   The Simulink subsystems call this function (extrinsic) so the model
%   executes exactly the same perception / planning / avoidance code as
%   runSimulation. The plant, wheel controller and safety supervisor are
%   implemented natively inside the Simulink model.
%
%   amrSimBridge('init', sc, controller, cfg, opts) -> [startPose, done]
%   amrSimBridge('world', t)                       -> dynObs (1 x 3*maxDyn: x,y,r)
%   amrSimBridge('sense', poseTrue)                -> ranges (1 x numBeams, NaN = none)
%   amrSimBridge('perceive', poseEst, t)           -> [nTracks nMoving]
%   amrSimBridge('replan', t)                      -> [nWaypoints blocked replans]
%   amrSimBridge('command', poseEst, prevCmd)      -> [vReq wReq mode]
%   amrSimBridge('post', pose, vw, wheel, cmd, t)  -> done (0/1)
%   amrSimBridge('finish')                         -> [result, log]
    persistent ctx tStep
    maxDyn = 6;
    if isempty(ctx) && ~strcmp(action, 'init')
        error('amrSimBridge:notInitialised', ['The navigation context is not initialised. ', ...
            'Press Run in amrNavigation.slx (its InitFcn initialises it) or call runSimulinkModel.']);
    end
    switch action
        case 'init'
            sc = varargin{1}; controller = varargin{2}; cfg = varargin{3};
            opts = struct();
            if numel(varargin) >= 4, opts = varargin{4}; end
            ctx = simInit(sc, controller, cfg, opts);
            varargout = {ctx.robot.pose, double(ctx.done)};
        case 'world'
            tStep = tic;
            ctx = simAdvanceWorld(ctx, varargin{1});
            d = zeros(1, 3 * maxDyn);
            for i = 1:min(numel(ctx.world.dyn), maxDyn)
                d(3*i-2:3*i) = [ctx.world.dyn(i).pos ctx.world.dyn(i).r];
            end
            varargout = {d};
        case 'sense'
            ctx.robot.pose = reshape(varargin{1}, 1, 3);
            ctx = simScan(ctx);
            r = ctx.scan.ranges; r(isnan(r)) = -1;
            varargout = {r};
        case 'perceive'
            ctx.robot.poseEst = reshape(varargin{1}, 1, 3);
            ctx = simPerceive(ctx, varargin{2});
            varargout = {[numel(ctx.tracks.list) size(movingTrackMatrix(ctx.tracks), 1)]};
        case 'replan'
            ctx = simReplan(ctx, varargin{1});
            varargout = {[size(ctx.path, 1) double(ctx.blockedNow) ctx.result.replans]};
        case 'command'
            ctx.robot.poseEst = reshape(varargin{1}, 1, 3);
            ctx.robot.cmd = reshape(varargin{2}, 1, 2);
            [ctx, cmd] = simCommand(ctx);
            varargout = {[cmd ctx.mode]};
        case 'post'
            ctx.robot.pose = reshape(varargin{1}, 1, 3);
            ctx.robot.poseEst = ctx.robot.pose;          % State Estimator: true pose
            vw = varargin{2};
            ctx.robot.v = vw(1); ctx.robot.w = vw(2);
            ctx.robot.wheel = reshape(varargin{3}, 1, 2);
            cmd = reshape(varargin{4}, 1, 2);
            ctx.robot.cmd = cmd;
            t = varargin{5};
            k = round(t / ctx.cfg.dt) + 1;
            st = NaN; if ~isempty(tStep), st = toc(tStep); end
            ctx = simPostStep(ctx, k, t, cmd, st);
            varargout = {double(ctx.done)};
        case 'finish'
            [result, log] = simFinalize(ctx);
            varargout = {result, log};
        case 'context'
            varargout = {ctx};
        otherwise
            error('amrSimBridge:action', 'Unknown action %s', action);
    end
end
