function v = t2v(T)
    assert(all(size(T) == [3, 3]), 't2v expects a 3x3 matrix');

    x = T(1, 3);
    y = T(2, 3);
    theta = atan2(T(2, 1), T(1, 1));

    v = [x; y; theta];
end
