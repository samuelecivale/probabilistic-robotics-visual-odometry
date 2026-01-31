function P = projection_matrix(K, T_cw)
  R = T_cw(1:3, 1:3);
  t = T_cw(1:3, 4);
  P = K * [R, t];
endfunction

