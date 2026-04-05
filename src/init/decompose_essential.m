function candidates = decompose_essential(E)
    assert(all(size(E) == [3, 3]), 'E must be 3x3');

    [U, ~, V] = svd(E);

    if det(U) < 0
        U(:,3) = -U(:,3);
    end
    if det(V) < 0
        V(:,3) = -V(:,3);
    end

    W = [0 -1 0;
         1  0 0;
         0  0 1];

    R1 = U * W * V';
    R2 = U * W' * V';
    t = U(:,3);

    if det(R1) < 0
        R1 = -R1;
        t = -t;
    end
    if det(R2) < 0
        R2 = -R2;
    end

    candidates = cell(4,1);
    candidates{1} = struct('R', R1, 't',  t);
    candidates{2} = struct('R', R1, 't', -t);
    candidates{3} = struct('R', R2, 't',  t);
    candidates{4} = struct('R', R2, 't', -t);
end
