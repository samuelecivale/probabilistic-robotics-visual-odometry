function [map, stats] = add_new_landmarks_from_frame_pair(map, meas_prev, meas_curr, T_prev, T_curr, camera, max_desc_dist, max_reproj_err)
    P_prev = camera.K * T_prev(1:3, :);
    P_curr = camera.K * T_curr(1:3, :);

    pair_matches = match_by_appearance_mutual_threshold(meas_prev.appearance, meas_curr.appearance, max_desc_dist);

    added = 0;
    candidate_count = numel(pair_matches.idx1);

    for k = 1:candidate_count
        i_prev = pair_matches.idx1(k);
        i_curr = pair_matches.idx2(k);

        desc = meas_curr.appearance(i_curr, :);

        % skip if descriptor already exists in map
        already_in_map = false;
        for m = 1:size(map.desc, 1)
            if appearance_distance(map.desc(m, :), desc) <= max_desc_dist
                already_in_map = true;
                break;
            end
        end

        if already_in_map
            continue;
        end

        uv_prev = meas_prev.uv(i_prev, :);
        uv_curr = meas_curr.uv(i_curr, :);

        pw = triangulate_point_linear(P_prev, P_curr, uv_prev, uv_curr);
        Xh = [pw, 1].';

        pc_prev = T_prev * Xh;
        pc_curr = T_curr * Xh;

        if pc_prev(3) <= camera.z_near || pc_curr(3) <= camera.z_near
            continue;
        end

        [uv_prev_proj, valid_prev] = project_point(camera.K, T_prev, pw, ...
            camera.width, camera.height, camera.z_near, 1e9);
        [uv_curr_proj, valid_curr] = project_point(camera.K, T_curr, pw, ...
            camera.width, camera.height, camera.z_near, 1e9);

        if ~(valid_prev && valid_curr)
            continue;
        end

        err_prev = norm(uv_prev_proj(:) - uv_prev(:));
        err_curr = norm(uv_curr_proj(:) - uv_curr(:));

        if err_prev > max_reproj_err || err_curr > max_reproj_err
            continue;
        end

        map.points(end+1, :) = pw;
        map.desc(end+1, :) = desc;
        map.obs_count(end+1, 1) = 2;

        added = added + 1;
    end

    stats = struct();
    stats.candidate_matches = candidate_count;
    stats.added_points = added;
end
