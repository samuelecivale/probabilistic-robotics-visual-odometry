function pw = triangulate_point_linear(P1, P2, uv1, uv2)
    assert(all(size(P1) == [3, 4]), 'P1 must be 3x4');
    assert(all(size(P2) == [3, 4]), 'P2 must be 3x4');
    assert(numel(uv1) == 2, 'uv1 must be 2D');
    assert(numel(uv2) == 2, 'uv2 must be 2D');

    u1 = uv1(1);
    v1 = uv1(2);
    u2 = uv2(1);
    v2 = uv2(2);

    A = [
        u1 * P1(3,:) - P1(1,:);
        v1 * P1(3,:) - P1(2,:);
        u2 * P2(3,:) - P2(1,:);
        v2 * P2(3,:) - P2(2,:)
    ];

    [~, ~, V] = svd(A);
    X = V(:, end);
    X = X ./ X(4);

    pw = X(1:3).';
end
