function T = pose2d_to_se3(pose)
    assert(numel(pose) == 3, 'pose2d_to_se3 expects [x y theta]');

    x = pose(1);
    y = pose(2);
    theta = pose(3);

    c = cos(theta);
    s = sin(theta);

    T = eye(4);
    T(1:3, 1:3) = [c -s  0;
                   s  c  0;
                   0  0  1];
    T(1:3, 4) = [x; y; 0];
end
