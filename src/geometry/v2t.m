function T = v2t(v)
    assert(numel(v) == 3, 'v2t expects a 3-vector [x y theta]');

    x = v(1);
    y = v(2);
    theta = v(3);

    c = cos(theta);
    s = sin(theta);

    T = [c -s x;
         s  c y;
         0  0 1];
end
