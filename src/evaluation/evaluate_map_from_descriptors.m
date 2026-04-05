function eval_map = evaluate_map_from_descriptors(map, world, T_wc0_gt, scale_ratio, max_desc_dist)
    if nargin < 5
        max_desc_dist = 1e-8;
    end

    matches = match_by_appearance_mutual_threshold(map.desc, world.appearance, max_desc_dist);

    map_pts = map.points(matches.idx1, :);
    world_pts = world.position(matches.idx2, :);

    T_c0w_gt = inv(T_wc0_gt);

    n = size(world_pts, 1);
    gt_pts_c0 = zeros(n, 3);

    for i = 1:n
        Xh = [world_pts(i, :) 1].';
        Xc0 = T_c0w_gt * Xh;
        gt_pts_c0(i, :) = Xc0(1:3).';
    end

    map_pts_scaled = scale_ratio * map_pts;
    errors = vecnorm((map_pts_scaled - gt_pts_c0).', 2, 1);
    rmse = sqrt(mean(errors .^ 2));

    eval_map = struct();
    eval_map.num_matches = numel(matches.idx1);
    eval_map.rmse = rmse;
    eval_map.mean_err = mean(errors);
    eval_map.median_err = median(errors);
    eval_map.errors = errors(:);
    eval_map.map_pts_scaled = map_pts_scaled;
    eval_map.gt_pts_c0 = gt_pts_c0;
end
