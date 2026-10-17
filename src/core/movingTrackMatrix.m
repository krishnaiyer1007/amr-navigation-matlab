function D = movingTrackMatrix(tracks)
%MOVINGTRACKMATRIX Kx5 [x y vx vy r] matrix of tracks flagged as moving.
    L = tracks.list;
    D = zeros(0, 5);
    for i = 1:numel(L)
        if L(i).moving
            D(end+1, :) = [L(i).pos L(i).vel L(i).r]; %#ok<AGROW>
        end
    end
end
