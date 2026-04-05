function matches = match_by_appearance_mutual_threshold(desc1, desc2, max_dist)
    forward = match_by_appearance_nn(desc1, desc2);
    backward = match_by_appearance_nn(desc2, desc1);

    matches = struct();
    matches.idx1 = zeros(0,1);
    matches.idx2 = zeros(0,1);
    matches.dist = zeros(0,1);

    if isempty(forward.idx1) || isempty(backward.idx1)
        return;
    end

    keep = false(numel(forward.idx1), 1);

    for k = 1:numel(forward.idx1)
        i = forward.idx1(k);
        j = forward.idx2(k);

        if j > 0 && backward.idx2(j) == i && forward.dist(k) <= max_dist
            keep(k) = true;
        end
    end

    matches.idx1 = forward.idx1(keep);
    matches.idx2 = forward.idx2(keep);
    matches.dist = forward.dist(keep);
end

