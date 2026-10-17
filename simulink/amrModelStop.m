function amrModelStop()
%AMRMODELSTOP StopFcn of amrNavigation.slx: collects the result after a run.
%   Saves amrResult and amrLog to the base workspace. After a run started
%   from the Run button (amrShowResults absent or true) it prints the
%   outcome and opens the trajectory and clearance figures.
    try
        [result, log] = amrSimBridge('finish');
    catch err
        warning('amrNavigation:finish', 'No run to collect: %s', err.message);
        return;
    end
    assignin('base', 'amrResult', result);
    assignin('base', 'amrLog', log);
    show = true;
    if evalin('base', 'exist(''amrShowResults'', ''var'')')
        show = evalin('base', 'amrShowResults');
    end
    if ~show, return; end
    fprintf(['amrNavigation result: %s | time %.1f s | path %.2f m | min clearance %.3f m | ', ...
             'replans %d\n'], result.reason, result.time, result.pathLength, result.minClearance, result.replans);
    ctx = amrSimBridge('context');
    figs = plotRun(ctx.sc, result, log, ctx.cfg);
    keep = [2 4];                                  % trajectory, clearance
    close(figs(setdiff(1:numel(figs), keep(keep <= numel(figs)))));
end
