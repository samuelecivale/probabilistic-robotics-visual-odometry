function matches = match_by_appearance_nn(desc1, desc2)
    assert(size(desc1, 2) == size(desc2, 2), 'Descriptor dimensions must match');

    n1 = size(desc1, 1);
    n2 = size(desc2, 1);

    matches = struct();
    matches.idx1 = zeros(0,1);
    matches.idx2 = zeros(0,1);
    matches.dist = zeros(0,1);

    if n1 == 0 || n2 == 0
        return;
    end

    s1 = sum(desc1.^2, 2);
    s2 = sum(desc2.^2, 2);

    D2 = bsxfun(@plus, s1, s2.') - 2 * (desc1 * desc2.');
    D2(D2 < 0) = 0;

    [best_d2, best_j] = min(D2, [], 2);

    matches.idx1 = (1:n1).';
    matches.idx2 = best_j;
    matches.dist = sqrt(best_d2);
end
