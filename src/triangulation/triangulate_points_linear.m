function Pw = triangulate_points_linear(P1, P2, UV1, UV2)
    assert(size(UV1, 2) == 2, 'UV1 must be Nx2');
    assert(size(UV2, 2) == 2, 'UV2 must be Nx2');
    assert(size(UV1, 1) == size(UV2, 1), 'UV1 and UV2 must have same number of rows');

    n = size(UV1, 1);
    Pw = zeros(n, 3);

    for i = 1:n
        Pw(i, :) = triangulate_point_linear(P1, P2, UV1(i, :), UV2(i, :));
    end
end
