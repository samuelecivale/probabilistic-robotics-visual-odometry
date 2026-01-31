function [T_cw, info] = solve_pose_projective_icp(K, T_init_cw, P_w, uv_meas, max_iters)
  T_cw = T_init_cw;

  info.iters = 0;
  info.final_rmse = NaN;

  lambda = 1e-3;      % damping iniziale un po' più alto
  prev_rmse = Inf;

  for it=1:max_iters
    info.iters = it;

    % residual corrente
    r0 = reproj_residual(K, T_cw, P_w, uv_meas);
    rmse0 = sqrt(mean(r0.^2));

    % jacobiano numerico attorno a xi=0
    fun = @(xi) reproj_residual(K, se3_exp(xi) * T_cw, P_w, uv_meas);
    J = numeric_jacobian(fun, zeros(6,1));

    H = J' * J;
    g = J' * r0;

    % STEP CORRETTO: -inv(H)*g
    dx = - (H + lambda*eye(6)) \ g;

    % clamp step per evitare salti enormi (stabilizza molto)
    max_step = 1e-1;
    if norm(dx) > max_step
      dx = dx * (max_step / norm(dx));
    endif

    % prova aggiornamento
    T_trial = se3_exp(dx) * T_cw;
    r_trial = reproj_residual(K, T_trial, P_w, uv_meas);
    rmse_trial = sqrt(mean(r_trial.^2));

    % semplice logica tipo LM:
    if rmse_trial < rmse0
      % accetta passo, riduci lambda
      T_cw = T_trial;
      lambda = lambda * 0.5;
      prev_rmse = rmse_trial;
    else
      % rifiuta, aumenta lambda e riprova al prossimo iter
      lambda = lambda * 5.0;
    endif

    % stop se miglioramento minimo
    if abs(rmse0 - prev_rmse) < 1e-6
      break;
    endif
  endfor

  r_final = reproj_residual(K, T_cw, P_w, uv_meas);
  info.final_rmse = sqrt(mean(r_final.^2));
endfunction

