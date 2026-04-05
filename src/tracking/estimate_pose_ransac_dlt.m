function result = estimate_pose_ransac_dlt(K, Pw, uv, params)
    if nargin < 4
        params = struct();
    end

    if ~isfield(params, 'max_iters');     params.max_iters = 200;    end
    if ~isfield(params, 'sample_size');   params.sample_size = 6;    end
    if ~isfield(params, 'inlier_thresh'); params.inlier_thresh = 3.0; end
    if ~isfield(params, 'min_inliers');   params.min_inliers = 10;   end
    if ~isfield(params, 'min_depth');     params.min_depth = 1e-8;   end

    n = size(Pw, 1);

    result = struct();
    result.success = false;
    result.T = eye(4);
    result.inliers = false(n, 1);
    result.num_inliers = 0;
    result.mean_err = Inf;
    result.iters = 0;

    if n < params.sample_size
        return;
    end

    best_num_inliers = 0;
    best_mean_err = Inf;
    best_T = eye(4);
    best_inliers = false(n, 1);

    for it = 1:params.max_iters
        sample_idx = randperm(n, params.sample_size);

        try
            xn_sample = pixels_to_normalized(K, uv(sample_idx, :));
            pose_sample = estimate_pose_dlt(Pw(sample_idx, :), xn_sample);
            T_sample = pose_to_matrix(pose_sample.R, pose_sample.t);
        catch
            continue;
        end

        [~, ~, err, valid] = reprojection_errors_pose(K, T_sample, Pw, uv, params.min_depth);
        inliers = valid & (err < params.inlier_thresh);
        num_inliers = sum(inliers);

        if num_inliers == 0
            continue;
        end

        mean_err = mean(err(inliers));

        better = false;
        if num_inliers > best_num_inliers
            better = true;
        elseif num_inliers == best_num_inliers && mean_err < best_mean_err
            better = true;
        end

        if better
            best_num_inliers = num_inliers;
            best_mean_err = mean_err;
            best_T = T_sample;
            best_inliers = inliers;
        end
    end

    result.iters = params.max_iters;

    if best_num_inliers < max(params.min_inliers, params.sample_size)
        return;
    end

    try
        xn_in = pixels_to_normalized(K, uv(best_inliers, :));
        pose_refit = estimate_pose_dlt(Pw(best_inliers, :), xn_in);
        T_refit = pose_to_matrix(pose_refit.R, pose_refit.t);

        [~, ~, err_refit, valid_refit] = reprojection_errors_pose(K, T_refit, Pw, uv, params.min_depth);
        inliers_refit = valid_refit & (err_refit < params.inlier_thresh);
        num_inliers_refit = sum(inliers_refit);

        if num_inliers_refit >= best_num_inliers
            best_T = T_refit;
            best_inliers = inliers_refit;
            best_num_inliers = num_inliers_refit;
            if num_inliers_refit > 0
                best_mean_err = mean(err_refit(inliers_refit));
            else
                best_mean_err = Inf;
            end
        end
    catch
        % keep previous best if refit fails
    end

    result.success = true;
    result.T = best_T;
    result.inliers = best_inliers;
    result.num_inliers = best_num_inliers;
    result.mean_err = best_mean_err;
end
