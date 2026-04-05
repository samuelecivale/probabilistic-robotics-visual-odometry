function [uv, is_valid, pc] = project_point(K, T_cw, pw, width, height, z_near, z_far)
    assert(all(size(K) == [3, 3]), 'K must be 3x3');
    assert(all(size(T_cw) == [4, 4]), 'T_cw must be 4x4');
    assert(numel(pw) == 3, 'pw must be a 3-vector');

    pw_h = [pw(:); 1];
    pc_h = T_cw * pw_h;
    pc = pc_h(1:3);

    z = pc(3);

    if z <= z_near || z >= z_far
        uv = [NaN; NaN];
        is_valid = false;
        return;
    end

    proj = K * (pc / z);
    u = proj(1);
    v = proj(2);

    uv = [u; v];

    is_valid = (u >= 0) && (u <= width) && (v >= 0) && (v <= height);
end
