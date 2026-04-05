function T_new = pose_update_left(T, delta)
    assert(all(size(T) == [4, 4]), 'T must be 4x4');
    assert(numel(delta) == 6, 'delta must be 6D');

    delta = delta(:);
    dt = delta(1:3);
    dw = delta(4:6);

    R_inc = rodrigues(dw);

    T_inc = eye(4);
    T_inc(1:3, 1:3) = R_inc;
    T_inc(1:3, 4) = dt;

    T_new = T_inc * T;
end
