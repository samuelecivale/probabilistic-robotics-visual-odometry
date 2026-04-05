function E = estimate_essential_eight_point(x1, x2)
    assert(size(x1, 2) == 3, 'x1 must be Nx3 homogeneous normalized points');
    assert(size(x2, 2) == 3, 'x2 must be Nx3 homogeneous normalized points');
    assert(size(x1, 1) == size(x2, 1), 'x1 and x2 must have same number of points');
    assert(size(x1, 1) >= 8, 'Need at least 8 correspondences');

    n = size(x1, 1);
    A = zeros(n, 9);

    for i = 1:n
        X1 = x1(i, :);
        X2 = x2(i, :);

        A(i, :) = [
            X2(1)*X1(1), X2(1)*X1(2), X2(1)*X1(3), ...
            X2(2)*X1(1), X2(2)*X1(2), X2(2)*X1(3), ...
            X2(3)*X1(1), X2(3)*X1(2), X2(3)*X1(3)
        ];
    end

    [~, ~, V] = svd(A);
    E = reshape(V(:, end), 3, 3).';

    [U, S, V] = svd(E);
    s = (S(1,1) + S(2,2)) / 2;
    E = U * diag([s, s, 0]) * V';

    if det(U) < 0
        U(:,3) = -U(:,3);
    end
    if det(V) < 0
        V(:,3) = -V(:,3);
    end
end
