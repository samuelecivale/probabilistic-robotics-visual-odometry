function result = select_essential_pose(candidates, x1, x2)
    assert(iscell(candidates), 'candidates must be a cell array');
    assert(size(x1, 2) == 3 && size(x2, 2) == 3, 'x1 and x2 must be Nx3');

    P1 = [eye(3), zeros(3,1)];

    best_count = -1;
    best_result = struct();

    for c = 1:numel(candidates)
        R = candidates{c}.R;
        t = candidates{c}.t;

        P2 = [R, t];

        n = size(x1, 1);
        Pw = zeros(n, 3);
        valid = false(n, 1);

        for i = 1:n
            pw = triangulate_point_linear(P1, P2, x1(i,1:2), x2(i,1:2));
            Pw(i, :) = pw;

            Xh = [pw, 1].';
            pc1 = P1 * Xh;
            pc2 = P2 * Xh;

            valid(i) = (pc1(3) > 0) && (pc2(3) > 0);
        end

        count = sum(valid);

        if count > best_count
            best_count = count;
            best_result.R = R;
            best_result.t = t;
            best_result.P1 = P1;
            best_result.P2 = P2;
            best_result.Pw = Pw;
            best_result.valid_cheirality = valid;
            best_result.num_valid = count;
        end
    end

    result = best_result;
end
