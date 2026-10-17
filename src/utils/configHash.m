function h = configHash(cfg)
%CONFIGHASH Short deterministic checksum of a configuration struct.
%   Used to tag test and benchmark outputs (not cryptographic).
    s = struct2text(cfg, '');
    b = double(s);
    h1 = mod(sum(b .* (1:numel(b))), 2^31 - 1);
    h2 = mod(sum(b .^ 2), 2^31 - 1);
    h = sprintf('%08x%08x', h1, h2);
end

function s = struct2text(x, prefix)
    s = '';
    f = fieldnames(x);
    for i = 1:numel(f)
        v = x.(f{i});
        name = [prefix '.' f{i}];
        if isstruct(v)
            s = [s struct2text(v, name)]; %#ok<AGROW>
        elseif ischar(v)
            s = [s name '=' v ';']; %#ok<AGROW>
        else
            s = [s name '=' sprintf('%.10g,', double(v(:))) ';']; %#ok<AGROW>
        end
    end
end
