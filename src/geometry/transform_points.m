function Q = transform_points(T, P)
    assert(all(size(T) == [4, 4]), 'T must be 4x4');
    assert(size(P, 2) == 3, 'P must be Nx3');

    n = size(P, 1);
    if n == 0
        Q = zeros(0, 3);
        return;
    end

    Ph = [P, ones(n, 1)].';
    Qh = T * Ph;
    Q = Qh(1:3, :).';
end
