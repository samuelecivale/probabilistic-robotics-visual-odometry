function T = pose_to_matrix(R, t)
    assert(all(size(R) == [3, 3]), 'R must be 3x3');
    assert(numel(t) == 3, 't must be 3D');

    T = eye(4);
    T(1:3, 1:3) = R;
    T(1:3, 4) = t(:);
end
