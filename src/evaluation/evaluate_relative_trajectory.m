function eval_traj = evaluate_relative_trajectory(T_est_all, T_wc_gt)
    n = size(T_est_all, 3);

    rot_trace_err = NaN(n-1, 1);
    scale_ratio = NaN(n-1, 1);
    valid_pair = false(n-1, 1);

    for k = 1:(n-1)
        T_est_0 = T_est_all(:, :, k);
        T_est_1 = T_est_all(:, :, k+1);

        if any(isnan(T_est_0(:))) || any(isnan(T_est_1(:)))
            continue;
        end

        % Estimated relative motion: current wrt previous
        rel_est = T_est_1 * inv(T_est_0);

        % GT relative camera motion: current wrt previous
        T_wc_0 = T_wc_gt(:, :, k);
        T_wc_1 = T_wc_gt(:, :, k+1);
        rel_gt = inv(T_wc_1) * T_wc_0;

        err_T = inv(rel_est) * rel_gt;
        rot_trace_err(k) = trace(eye(3) - err_T(1:3, 1:3));

        gt_norm = norm(rel_gt(1:3, 4));
        est_norm = norm(rel_est(1:3, 4));

        if gt_norm > 1e-12
            scale_ratio(k) = est_norm / gt_norm;
        end

        valid_pair(k) = true;
    end

    valid_scale = valid_pair & ~isnan(scale_ratio) & isfinite(scale_ratio);

    eval_traj = struct();
    eval_traj.rot_trace_err = rot_trace_err;
    eval_traj.scale_ratio = scale_ratio;
    eval_traj.valid_pair = valid_pair;
    eval_traj.valid_scale = valid_scale;

    if any(valid_pair)
        eval_traj.mean_rot_trace_err = mean(rot_trace_err(valid_pair));
        eval_traj.median_rot_trace_err = median(rot_trace_err(valid_pair));
    else
        eval_traj.mean_rot_trace_err = NaN;
        eval_traj.median_rot_trace_err = NaN;
    end

    if any(valid_scale)
        eval_traj.mean_scale_ratio = mean(scale_ratio(valid_scale));
        eval_traj.median_scale_ratio = median(scale_ratio(valid_scale));
        eval_traj.std_scale_ratio = std(scale_ratio(valid_scale));
    else
        eval_traj.mean_scale_ratio = NaN;
        eval_traj.median_scale_ratio = NaN;
        eval_traj.std_scale_ratio = NaN;
    end
end
