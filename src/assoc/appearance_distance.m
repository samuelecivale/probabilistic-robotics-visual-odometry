function d = appearance_distance(a, b)
    assert(isvector(a) && isvector(b), 'appearance_distance expects two vectors');
    assert(numel(a) == numel(b), 'Descriptor sizes must match');

    diff = a(:) - b(:);
    d = norm(diff);
end
