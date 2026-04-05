function pose = estimate_pose_dlt(Pw, xn)
    assert(size(Pw, 2) == 3, 'Pw must be Nx3');
    assert(size(xn, 2) == 3, 'xn must be Nx3 homogeneous normalized points');
    assert(size(Pw, 1) == size(xn, 1), 'Pw and xn must have same number of rows');
    assert(size(Pw, 1) >= 6, 'Need at least 6 correspondences');

    n = size(Pw, 1);
    A = zeros(2*n, 12);

    for i = 1:n
        X = [Pw(i, :) 1];
        x = xn(i, 1);
        y = xn(i, 2);

        A(2*i-1, :) = [0 0 0 0, -X, y*X];
        A(2*i,   :) = [X, 0 0 0 0, -x*X];
    end

    [~, ~, V] = svd(A);
    P = reshape(V(:, end), 4, 3).';

    M = P(:, 1:3);
    tt = P(:, 4);

    if det(M) < 0
        P = -P;
        M = -M(:, :);
        tt = -tt;
    end

    scale = mean([norm(M(1,:)), norm(M(2,:)), norm(M(3,:))]);

    R_approx = M / scale;
    t = tt / scale;

    [U, ~, V] = svd(R_approx);
    R = U * V';

    if det(R) < 0
        R = -R;
        t = -t;
    end

    pose = struct();
    pose.R = R;
    pose.t = t;
    pose.P = [R, t];
end
