function xn = pixels_to_normalized(K, uv)
    assert(all(size(K) == [3, 3]), 'K must be 3x3');
    assert(size(uv, 2) == 2, 'uv must be Nx2');

    n = size(uv, 1);
    uv_h = [uv, ones(n,1)].';
    xn_h = K \ uv_h;
    xn = xn_h.';
end
