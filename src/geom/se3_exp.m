function T = se3_exp(xi)
  w = xi(1:3);
  v = xi(4:6);

  th = norm(w);
  T = eye(4);

  if th < 1e-12
    R = eye(3);
    V = eye(3);
  else
    W = skew(w);
    A = sin(th)/th;
    B = (1 - cos(th))/(th^2);
    C = (1 - A)/(th^2);
    R = eye(3) + A*W + B*(W*W);
    V = eye(3) + B*W + C*(W*W);
  endif

  T(1:3,1:3) = R;
  T(1:3,4) = V*v;
endfunction

