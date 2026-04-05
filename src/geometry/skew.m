function S = skew(v)
    assert(numel(v) == 3, 'skew expects a 3-vector');
    v = v(:);

    S = [   0   -v(3)  v(2);
          v(3)    0   -v(1);
         -v(2)  v(1)    0  ];
end
