function matches = match_by_appearance_nn(desc1, desc2)
    assert(size(desc1, 2) == size(desc2, 2), 'Descriptor dimensions must match');

    n1 = size(desc1, 1);
    n2 = size(desc2, 1);

    idx1 = zeros(n1, 1);
    idx2 = zeros(n1, 1);
    dist = zeros(n1, 1);

    for i = 1:n1
        best_j = 0;
        best_d = inf;

        for j = 1:n2
            d = appearance_distance(desc1(i, :), desc2(j, :));
            if d < best_d
                best_d = d;
                best_j = j;
            end
        end

        idx1(i) = i;
        idx2(i) = best_j;
        dist(i) = best_d;
    end

    matches = struct();
    matches.idx1 = idx1;
    matches.idx2 = idx2;
    matches.dist = dist;
end
