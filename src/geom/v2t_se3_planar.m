function T = v2t_se3_planar(p)
  x = p(1); y = p(2); th = p(3);
  c = cos(th); s = sin(th);

  T = eye(4);
  T(1:3,1:3) = [ c -s  0;
                 s  c  0;
                 0  0  1 ];
  T(1:3,4)   = [x; y; 0];
endfunction

