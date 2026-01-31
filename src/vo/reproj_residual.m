function r = reproj_residual(K, T_cw, P_w, uv_meas)
  uv_pred = project_points(K, T_cw, P_w);
  e = uv_meas - uv_pred;
  r = reshape(e, [], 1);
endfunction

