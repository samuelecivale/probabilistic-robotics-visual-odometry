function matches = match_by_appearance_mutual_threshold(desc1, desc2, max_dist)
    forward = match_by_appearance_nn(desc1, desc2);
    backward = match_by_appearance_nn(desc2, desc1);

    keep_idx1 = [];
    keep_idx2 = [];
    keep_dist = [];

    for k = 1:numel(forward.idx1)
        i = forward.idx1(k);
        j = forward.idx2(k);

        if j > 0 && backward.idx2(j) == i && forward.dist(k) <= max_dist
            keep_idx1(end+1, 1) = i;
            keep_idx2(end+1, 1) = j;
            keep_dist(end+1, 1) = forward.dist(k);
        end
    end

    matches = struct();
    matches.idx1 = keep_idx1;
    matches.idx2 = keep_idx2;
    matches.dist = keep_dist;
end
