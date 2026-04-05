function [map, stats] = add_new_landmarks_from_frame_pair(map, meas_prev, meas_curr, T_prev, T_curr, camera, max_desc_dist, max_reproj_err, matched_curr_idx)
    if nargin < 9
        matched_curr_idx = [];
    end

    P_prev = camera.K * T_prev(1:3, :);
    P_curr = camera.K * T_curr(1:3, :);

    pair_matches = match_by_appearance_mutual_threshold(meas_prev.appearance, meas_curr.appearance, max_desc_dist);

    if ~isempty(matched_curr_idx)
        unmatched_curr = true(size(meas_curr.appearance, 1), 1);
        unmatched_curr(matched_curr_idx) = false;

        keep = unmatched_curr(pair_matches.idx2);
        pair_matches.idx1 = pair_matches.idx1(keep);
        pair_matches.idx2 = pair_matches.idx2(keep);
        pair_matches.dist = pair_matches.dist(keep);
    end

    added = 0;
    candidate_count = numel(pair_matches.idx1);

    for k = 1:candidate_count
        i_prev = pair_matches.idx1(k);
        i_curr = pair_matches.idx2(k);

        uv_prev = meas_prev.uv(i_prev, :);
        uv_curr = meas_curr.uv(i_curr, :);

        pw = triangulate_point_linear(P_prev, P_curr, uv_prev, uv_curr);
        Xh = [pw, 1].';

        pc_prev = T_prev * Xh;
        pc_curr = T_curr * Xh;

        if pc_prev(3) <= 1e-8 || pc_curr(3) <= 1e-8
            continue;
        end

        [~, ~, err_prev, valid_prev] = reprojection_errors_pose(camera.K, T_prev, pw, uv_prev, 1e-8);
        [~, ~, err_curr, valid_curr] = reprojection_errors_pose(camera.K, T_curr, pw, uv_curr, 1e-8);

        if ~(valid_prev && valid_curr)
            continue;
        end

        if err_prev > max_reproj_err || err_curr > max_reproj_err
            continue;
        end

        map.points(end+1, :) = pw;
        map.desc(end+1, :) = meas_curr.appearance(i_curr, :);
        map.obs_count(end+1, 1) = 2;

        added = added + 1;
    end

    stats = struct();
    stats.candidate_matches = candidate_count;
    stats.added_points = added;
end
