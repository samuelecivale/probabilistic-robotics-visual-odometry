function R = rodrigues(w)
    assert(numel(w) == 3, 'rodrigues expects a 3-vector');
    w = w(:);

    theta = norm(w);

    if theta < 1e-12
        R = eye(3) + skew(w);
        return;
    end

    k = w / theta;
    K = skew(k);

    R = eye(3) + sin(theta) * K + (1 - cos(theta)) * (K * K);
end
