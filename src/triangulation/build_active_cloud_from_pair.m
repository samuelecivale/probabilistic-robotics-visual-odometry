function active = build_active_cloud_from_pair(camera, T_curr_prev, meas_prev, meas_curr, image_matches, max_reproj_err)
    if nargin < 6
        max_reproj_err = 2.0;
    end

    K = camera.K;
    P1 = K * [eye(3), zeros(3,1)];
    P2 = K * T_curr_prev(1:3, :);

    points_prev = zeros(0,3);
    points_curr = zeros(0,3);
    curr_img_idx = zeros(0,1);
    desc_curr = zeros(0,10);

    for k = 1:numel(image_matches.idx1)
        i_prev = image_matches.idx1(k);
        i_curr = image_matches.idx2(k);

        uv_prev = meas_prev.uv(i_prev, :);
        uv_curr = meas_curr.uv(i_curr, :);

        pw_prev = triangulate_point_linear(P1, P2, uv_prev, uv_curr);
        Xh = [pw_prev, 1].';

        pc_prev = Xh(1:3);
        pc_curr_h = T_curr_prev * Xh;
        pc_curr = pc_curr_h(1:3);

        if pc_prev(3) <= 1e-8 || pc_curr(3) <= 1e-8
            continue;
        end

        [~, ~, err_prev, valid_prev] = reprojection_errors_pose(K, eye(4), pw_prev, uv_prev, 1e-8);
        [~, ~, err_curr, valid_curr] = reprojection_errors_pose(K, T_curr_prev, pw_prev, uv_curr, 1e-8);

        if ~(valid_prev && valid_curr)
            continue;
        end

        if err_prev > max_reproj_err || err_curr > max_reproj_err
            continue;
        end

        pw_curr = transform_points(T_curr_prev, pw_prev);

        points_prev(end+1, :) = pw_prev;
        points_curr(end+1, :) = pw_curr;
        curr_img_idx(end+1, 1) = i_curr;
        desc_curr(end+1, :) = meas_curr.appearance(i_curr, :);
    end

    active = struct();
    active.points_prev = points_prev;
    active.points_curr = points_curr;
    active.curr_img_idx = curr_img_idx;
    active.desc_curr = desc_curr;
end
