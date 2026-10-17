function tr = initializeTracks()
%INITIALIZETRACKS Empty obstacle track list.
    tr.list = struct('id', {}, 'pos', {}, 'vel', {}, 'r', {}, 'lastSeen', {}, ...
                     'hits', {}, 'moving', {});
    tr.nextId = 1;
end
