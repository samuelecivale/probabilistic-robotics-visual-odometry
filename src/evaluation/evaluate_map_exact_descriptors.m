function eval_map = evaluate_map_exact_descriptors(map, world, T_wc0_gt, scale_correction, desc_tol)
    if nargin < 5
        desc_tol = 1e-10;
    end

    T_c0w_gt = inv(T_wc0_gt);

    matched_est = zeros(0,3);
    matched_gt = zeros(0,3);
    errors = zeros(0,1);

    for i = 1:size(map.points, 1)
        d = map.desc(i, :);

        for j = 1:size(world.appearance, 1)
            if all(abs(d - world.appearance(j, :)) <= desc_tol)
                Xw = [world.position(j, :) 1].';
                Xc0 = T_c0w_gt * Xw;

                p_est = scale_correction * map.points(i, :);
                p_gt = Xc0(1:3).';

                matched_est(end+1, :) = p_est;
                matched_gt(end+1, :) = p_gt;
                errors(end+1, 1) = norm(p_est - p_gt);
                break;
            end
        end
    end

    if isempty(errors)
        rmse = NaN;
        mean_err = NaN;
        median_err = NaN;
    else
        rmse = sqrt(mean(errors.^2));
        mean_err = mean(errors);
        median_err = median(errors);
    end

    eval_map = struct();
    eval_map.num_matches = numel(errors);
    eval_map.rmse = rmse;
    eval_map.mean_err = mean_err;
    eval_map.median_err = median_err;
    eval_map.errors = errors;
    eval_map.est_points = matched_est;
    eval_map.gt_points = matched_gt;
end
