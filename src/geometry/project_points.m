function [uvs, valid_mask, pcs] = project_points(K, T_cw, Pw, width, height, z_near, z_far)
    assert(size(Pw, 2) == 3, 'Pw must be Nx3');

    n = size(Pw, 1);
    uvs = NaN(n, 2);
    valid_mask = false(n, 1);
    pcs = NaN(n, 3);

    for i = 1:n
        [uv, is_valid, pc] = project_point(K, T_cw, Pw(i, :), width, height, z_near, z_far);
        uvs(i, :) = uv.';
        valid_mask(i) = is_valid;
        pcs(i, :) = pc.';
    end
end
