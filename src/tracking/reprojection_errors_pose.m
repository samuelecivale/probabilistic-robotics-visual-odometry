function [uv_proj, residuals, err_norm, valid] = reprojection_errors_pose(K, T, Pw, uv, min_depth)
    if nargin < 5
        min_depth = 1e-8;
    end

    assert(all(size(K) == [3, 3]), 'K must be 3x3');
    assert(all(size(T) == [4, 4]), 'T must be 4x4');
    assert(size(Pw, 2) == 3, 'Pw must be Nx3');
    assert(size(uv, 2) == 2, 'uv must be Nx2');
    assert(size(Pw, 1) == size(uv, 1), 'Pw and uv must have same number of rows');

    fx = K(1,1);
    fy = K(2,2);
    cx = K(1,3);
    cy = K(2,3);

    n = size(Pw, 1);

    uv_proj = NaN(n, 2);
    residuals = NaN(n, 2);
    err_norm = NaN(n, 1);
    valid = false(n, 1);

    for i = 1:n
        Xh = [Pw(i, :) 1].';
        pc = T * Xh;

        x = pc(1);
        y = pc(2);
        z = pc(3);

        if z <= min_depth
            continue;
        end

        u = fx * x / z + cx;
        v = fy * y / z + cy;

        uv_proj(i, :) = [u, v];
        residuals(i, :) = [u - uv(i,1), v - uv(i,2)];
        err_norm(i) = norm(residuals(i, :));
        valid(i) = true;
    end
end
