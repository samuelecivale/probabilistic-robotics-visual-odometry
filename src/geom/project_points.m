function uv = project_points(K, T_cw, P_w)
  N = columns(P_w);
  P_h = [P_w; ones(1, N)];
  P_c = T_cw * P_h;
  X = P_c(1,:); Y = P_c(2,:); Z = P_c(3,:);

  x = X ./ Z;
  y = Y ./ Z;

  p = K * [x; y; ones(1,N)];
  uv = p(1:2, :);
endfunction

