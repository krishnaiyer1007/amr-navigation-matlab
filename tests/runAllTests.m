function results = runAllTests()
%RUNALLTESTS Run every unit/integration test and save a dated report.
%   In MATLAB uses the unit test framework (runtests); in GNU Octave each
%   script is executed directly. Report: results/logs/test_report_<date>.txt
%   with MATLAB version, configuration hash and git commit (if available).
    root = setupPaths();
    names = {'testKinematics', 'testAStar', 'testCollision', 'testAvoidance', ...
             'testMapping', 'testReplanning', 'testDeterminism'};
    isOctave = exist('OCTAVE_VERSION', 'builtin') > 0;
    stamp = datestr(now, 'yyyy-mm-dd_HHMMSS');
    rep = fullfile(root, 'results', 'logs', ['test_report_' stamp '.txt']);
    fid = fopen(rep, 'w');
    hdr = sprintf('AMR navigation test report\nDate: %s\nVersion: %s\nConfig hash: %s\nCommit: %s\n\n', ...
        datestr(now), version, configHash(defaultConfig()), gitCommit(root));
    fprintf('%s', hdr); fprintf(fid, '%s', hdr);
    results = struct('name', names, 'passed', false, 'details', '');
    for i = 1:numel(names)
        t0 = tic;
        if ~isOctave
            r = runtests(names{i});
            ok = all([r.Passed]);
            det = sprintf('%d/%d sections passed', nnz([r.Passed]), numel(r));
            for j = find(~[r.Passed])
                det = sprintf('%s\n      FAILED: %s', det, r(j).Name);
            end
        else
            try
                evalin('base', names{i}); ok = true; det = 'all sections passed';
            catch err
                ok = false; det = err.message;
            end
        end
        results(i).passed = ok; results(i).details = det;
        line = sprintf('[%s] %-16s %6.1fs  %s\n', ternary(ok, 'PASS', 'FAIL'), names{i}, toc(t0), det);
        fprintf('%s', line); fprintf(fid, '%s', line);
    end
    s = sprintf('\n%d of %d test files passed.\n', nnz([results.passed]), numel(results));
    fprintf('%s', s); fprintf(fid, '%s', s); fclose(fid);
    fprintf('Report saved to %s\n', rep);
end

function out = ternary(c, a, b)
    if c, out = a; else, out = b; end
end

function c = gitCommit(root)
    c = 'n/a';
    try
        [st, o] = system(sprintf('git -C "%s" rev-parse --short HEAD', root));
        if st == 0, c = strtrim(o); end
    catch
    end
end
