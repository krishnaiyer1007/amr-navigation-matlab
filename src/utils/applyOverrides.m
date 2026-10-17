function s = applyOverrides(s, ov)
%APPLYOVERRIDES Recursively copy fields of OV into struct S.
%   Used for per-scenario and per-experiment configuration changes.
    if isempty(ov), return; end
    f = fieldnames(ov);
    for i = 1:numel(f)
        if isstruct(ov.(f{i})) && isfield(s, f{i}) && isstruct(s.(f{i}))
            s.(f{i}) = applyOverrides(s.(f{i}), ov.(f{i}));
        else
            s.(f{i}) = ov.(f{i});
        end
    end
end
