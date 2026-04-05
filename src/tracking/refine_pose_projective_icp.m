function result = refine_pose_projective_icp(K, T_init, Pw, uv, params)
    if nargin < 5
        params = struct();
    end

    if ~isfield(params, 'max_iters');   params.max_iters = 10;      end
    if ~isfield(params, 'lambda');      params.lambda = 1e-6;       end
    if ~isfield(params, 'huber_delta'); params.huber_delta = 5.0;   end
    if ~isfield(params, 'min_depth');   params.min_depth = 1e-8;    end
    if ~isfield(params, 'min_points');  params.min_points = 6;      end
    if ~isfield(params, 'step_tol');    params.step_tol = 1e-8;     end

    fx = K(1,1);
    fy = K(2,2);
    cx = K(1,3);
    cy = K(2,3);

    T = T_init;
    lambda = params.lambda;

    [~, ~, err0, valid0] = reprojection_errors_pose(K, T, Pw, uv, params.min_depth);

    if any(valid0)
        best_mean = mean(err0(valid0));
        best_valid = valid0;
        best_err = err0;
    else
        best_mean = Inf;
        best_valid = false(size(Pw,1),1);
        best_err = NaN(size(Pw,1),1);
    end

    T_best = T;
    last_step_norm = NaN;
    converged = false;
    used_points_last = 0;

    for iter = 1:params.max_iters
        J = [];
        r = [];
        used_points = 0;

        for i = 1:size(Pw, 1)
            Xh = [Pw(i, :) 1].';
            pc_h = T * Xh;
            pc = pc_h(1:3);

            x = pc(1);
            y = pc(2);
            z = pc(3);

            if z <= params.min_depth
                continue;
            end

            u = fx * x / z + cx;
            v = fy * y / z + cy;

            ri = [u - uv(i,1); v - uv(i,2)];
            ri_norm = norm(ri);

            wi = 1.0;
            if ri_norm > params.huber_delta
                wi = params.huber_delta / ri_norm;
            end

            J_proj = [fx / z,      0, -fx * x / (z * z);
                           0, fy / z, -fy * y / (z * z)];

            J_pose = [eye(3), -skew(pc)];
            Ji = J_proj * J_pose;

            sw = sqrt(wi);

            J = [J; sw * Ji];
            r = [r; sw * ri];

            used_points = used_points + 1;
        end

        used_points_last = used_points;

        if used_points < params.min_points
            break;
        end

        H = J' * J + lambda * eye(6);
        b = J' * r;

        delta = -H \ b;
        last_step_norm = norm(delta);

        if last_step_norm < params.step_tol
            converged = true;
            break;
        end

        T_candidate = pose_update_left(T, delta);
        [~, ~, err_cand, valid_cand] = reprojection_errors_pose(K, T_candidate, Pw, uv, params.min_depth);

        if any(valid_cand)
            mean_cand = mean(err_cand(valid_cand));
        else
            mean_cand = Inf;
        end

        if mean_cand <= best_mean
            T = T_candidate;
            T_best = T_candidate;
            best_mean = mean_cand;
            best_valid = valid_cand;
            best_err = err_cand;
            lambda = max(lambda / 2, 1e-12);
        else
            lambda = min(lambda * 10, 1e6);
        end
    end

    result = struct();
    result.T = T_best;
    result.mean_err = best_mean;
    result.err = best_err;
    result.valid = best_valid;
    result.num_valid = sum(best_valid);
    result.iters = iter;
    result.used_points = used_points_last;
    result.final_step_norm = last_step_norm;
    result.converged = converged;
    result.success = isfinite(best_mean) && (sum(best_valid) >= params.min_points);
end
