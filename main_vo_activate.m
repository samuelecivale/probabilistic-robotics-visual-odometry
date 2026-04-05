clear;
clc;
close all;

addpath(genpath('src'));

camera = load_camera(fullfile('data', 'camera.dat'));
traj   = load_trajectory(fullfile('data', 'trajectory.dat'));

num_frames = numel(traj.pose_id);
measurements = load_measurement_sequence('data', num_frames);

max_desc_dist = 1e-8;
max_cloud_reproj_err = 2.0;

ransac_params = struct();
ransac_params.max_iters = 200;
ransac_params.sample_size = 6;
ransac_params.inlier_thresh = 3.0;
ransac_params.min_inliers = 10;
ransac_params.min_depth = 1e-8;

refine_params = struct();
refine_params.max_iters = 10;
refine_params.lambda = 1e-6;
refine_params.huber_delta = 5.0;
refine_params.min_depth = 1e-8;
refine_params.min_points = 6;
refine_params.step_tol = 1e-8;

meas0 = measurements{1};
meas1 = measurements{2};

% --- Two-view init
matches01 = match_by_appearance_mutual_threshold(meas0.appearance, meas1.appearance, max_desc_dist);

uv0 = meas0.uv(matches01.idx1, :);
uv1 = meas1.uv(matches01.idx2, :);

x0 = pixels_to_normalized(camera.K, uv0);
x1 = pixels_to_normalized(camera.K, uv1);

E = estimate_essential_eight_point(x0, x1);
candidates = decompose_essential(E);
init_result = select_essential_pose(candidates, x0, x1);

valid01 = init_result.valid_cheirality;
Pw0 = init_result.Pw(valid01, :);  % points in frame c0

t01 = init_result.t;
t01 = t01 / norm(t01);
T_10 = pose_to_matrix(init_result.R, t01);  % transform c0 -> c1

% Active cloud for next step: points expressed in frame c1
active_points_prev = transform_points(T_10, Pw0);
active_prev_img_idx = matches01.idx2(valid01);
active_desc_prev = meas1.appearance(active_prev_img_idx, :); %#ok<NASGU>

% Global map stored in frame c0
map = struct();
map.points = Pw0;
map.desc = meas1.appearance(active_prev_img_idx, :);
map.obs_count = 2 * ones(size(Pw0, 1), 1);
map.created_frame = 2 * ones(size(Pw0, 1), 1);
map.last_seen = 2 * ones(size(Pw0, 1), 1);

T_est_all = NaN(4, 4, num_frames);
T_est_all(:, :, 1) = eye(4);
T_est_all(:, :, 2) = T_10;

active_size = zeros(num_frames, 1);
active_corr_count = zeros(num_frames, 1);
global_fallback_count = zeros(num_frames, 1);
inlier_count = zeros(num_frames, 1);
reproj_init_mean = NaN(num_frames, 1);
reproj_mean = NaN(num_frames, 1);
added_points = zeros(num_frames, 1);
map_size = zeros(num_frames, 1);
frame_status = cell(num_frames, 1);

active_size(1) = size(Pw0, 1);
active_size(2) = size(active_points_prev, 1);
map_size(1) = size(map.points, 1);
map_size(2) = size(map.points, 1);
frame_status{1} = 'init_identity';
frame_status{2} = 'init_two_view';

fprintf('=== ACTIVE-CLOUD VISUAL ODOMETRY ===\n');
fprintf('Initial active cloud size: %d\n', size(active_points_prev, 1));
fprintf('Initial global map size: %d\n', size(map.points, 1));
fprintf('frame %03d | status=%s | active=%d | map=%d\n', 0, frame_status{1}, active_size(1), map_size(1));
fprintf('frame %03d | status=%s | active=%d | map=%d\n', 1, frame_status{2}, active_size(2), map_size(2));

T_prev_c0 = T_10;
meas_prev = meas1;

for k = 3:num_frames
    meas_curr = measurements{k};

    % 1) image-image matches between previous and current frame
    image_matches = match_by_appearance_mutual_threshold(meas_prev.appearance, meas_curr.appearance, max_desc_dist);

    % 2) propagate them to active 3D cloud
    assoc = extract_active_correspondences(image_matches, active_prev_img_idx);
    active_corr_count(k) = numel(assoc.active_idx);

    use_global_fallback = (active_corr_count(k) < 6);

    if ~use_global_fallback
        Pw_corr = active_points_prev(assoc.active_idx, :);   % points in previous camera frame
        uv_corr = meas_curr.uv(assoc.curr_idx, :);

        ransac_result = estimate_pose_ransac_dlt(camera.K, Pw_corr, uv_corr, ransac_params);

        if ransac_result.success
            inlier_mask = ransac_result.inliers;
            T_init = ransac_result.T;
            Pw_pose = Pw_corr(inlier_mask, :);
            uv_pose = uv_corr(inlier_mask, :);
            matched_curr_idx_for_pose = assoc.curr_idx(inlier_mask);
            inlier_count(k) = sum(inlier_mask);
            frame_status{k} = 'ok_active_ransac';
        else
            xn_corr = pixels_to_normalized(camera.K, uv_corr);
            pose_init = estimate_pose_dlt(Pw_corr, xn_corr);
            T_init = pose_to_matrix(pose_init.R, pose_init.t);

            Pw_pose = Pw_corr;
            uv_pose = uv_corr;
            matched_curr_idx_for_pose = assoc.curr_idx;
            inlier_count(k) = size(Pw_corr, 1);
            frame_status{k} = 'ok_active_fallback';
        end

        [~, ~, err_init, valid_init] = reprojection_errors_pose(camera.K, T_init, Pw_pose, uv_pose, 1e-8);
        if any(valid_init)
            reproj_init_mean(k) = mean(err_init(valid_init));
        end

        refine_result = refine_pose_projective_icp(camera.K, T_init, Pw_pose, uv_pose, refine_params);
        T_curr_prev = refine_result.T;
        T_curr_c0 = T_curr_prev * T_prev_c0;

        if refine_result.success
            reproj_mean(k) = refine_result.mean_err;
        end

    else
        % Fallback: use global map in frame c0
        global_matches = match_by_appearance_mutual_threshold(map.desc, meas_curr.appearance, max_desc_dist);
        global_fallback_count(k) = numel(global_matches.idx1);

        if numel(global_matches.idx1) < 6
            frame_status{k} = 'too_few_matches';
            T_est_all(:, :, k) = NaN(4);
            map_size(k) = size(map.points, 1);
            active_size(k) = size(active_points_prev, 1);

            fprintf('frame %03d | status=%s | active_corr=%d | global_matches=%d | map=%d\n', ...
                k-1, frame_status{k}, active_corr_count(k), global_fallback_count(k), map_size(k));

            meas_prev = meas_curr;
            continue;
        end

        Pw_corr = map.points(global_matches.idx1, :);   % points in c0
        uv_corr = meas_curr.uv(global_matches.idx2, :);

        ransac_result = estimate_pose_ransac_dlt(camera.K, Pw_corr, uv_corr, ransac_params);

        if ransac_result.success
            inlier_mask = ransac_result.inliers;
            T_init = ransac_result.T;
            Pw_pose = Pw_corr(inlier_mask, :);
            uv_pose = uv_corr(inlier_mask, :);
            matched_curr_idx_for_pose = global_matches.idx2(inlier_mask);
            inlier_count(k) = sum(inlier_mask);
            frame_status{k} = 'ok_global_ransac';
        else
            xn_corr = pixels_to_normalized(camera.K, uv_corr);
            pose_init = estimate_pose_dlt(Pw_corr, xn_corr);
            T_init = pose_to_matrix(pose_init.R, pose_init.t);

            Pw_pose = Pw_corr;
            uv_pose = uv_corr;
            matched_curr_idx_for_pose = global_matches.idx2;
            inlier_count(k) = size(Pw_corr, 1);
            frame_status{k} = 'ok_global_fallback';
        end

        [~, ~, err_init, valid_init] = reprojection_errors_pose(camera.K, T_init, Pw_pose, uv_pose, 1e-8);
        if any(valid_init)
            reproj_init_mean(k) = mean(err_init(valid_init));
        end

        refine_result = refine_pose_projective_icp(camera.K, T_init, Pw_pose, uv_pose, refine_params);
        T_curr_c0 = refine_result.T;
        T_curr_prev = T_curr_c0 * inv(T_prev_c0);

        if refine_result.success
            reproj_mean(k) = refine_result.mean_err;
        end
    end

    T_est_all(:, :, k) = T_curr_c0;

    % 3) Build next active cloud from previous/current pair
    active = build_active_cloud_from_pair(camera, T_curr_prev, meas_prev, meas_curr, image_matches, max_cloud_reproj_err);

    active_points_prev = active.points_curr;
    active_prev_img_idx = active.curr_img_idx;
    active_size(k) = size(active_points_prev, 1);

    % 4) Update global map with newly triangulated points expressed in c0
    points_c0_new = transform_points(inv(T_prev_c0), active.points_prev);
    [map, map_stats] = update_global_map_unique(map, points_c0_new, active.desc_curr, k, max_desc_dist);
    added_points(k) = map_stats.added;
    map_size(k) = size(map.points, 1);

    if k <= 10 || mod(k-1, 5) == 0 || k == num_frames
        fprintf('frame %03d | status=%s | active_corr=%d | global_fb=%d | inliers=%d | active=%d | init_reproj=%.6f px | refined_reproj=%.6f px | added=%d | map=%d\n', ...
            k-1, frame_status{k}, active_corr_count(k), global_fallback_count(k), inlier_count(k), ...
            active_size(k), reproj_init_mean(k), reproj_mean(k), added_points(k), map_size(k));
    end

    T_prev_c0 = T_curr_c0;
    meas_prev = meas_curr;
end

num_ok = 0;
for k = 1:num_frames
    if ~isempty(frame_status{k}) && ~strcmp(frame_status{k}, 'too_few_matches')
        num_ok = num_ok + 1;
    end
end

fprintf('\nProcessed frames: %d\n', num_frames);
fprintf('Successful frames: %d\n', num_ok);
fprintf('Final active cloud size: %d\n', size(active_points_prev, 1));
fprintf('Final global map size: %d\n', size(map.points, 1));
fprintf('Average active correspondences: %.2f\n', mean(active_corr_count(active_corr_count > 0)));

if any(global_fallback_count > 0)
    fprintf('Average global fallback matches: %.2f\n', mean(global_fallback_count(global_fallback_count > 0)));
else
    fprintf('Average global fallback matches: 0.00\n');
end

if any(inlier_count > 0)
    fprintf('Average inliers: %.2f\n', mean(inlier_count(inlier_count > 0)));
else
    fprintf('Average inliers: 0.00\n');
end

valid_reproj = ~isnan(reproj_mean);
if any(valid_reproj)
    fprintf('Average refined reprojection error: %.6f px\n', mean(reproj_mean(valid_reproj)));
end

if exist('results', 'dir') ~= 7
    mkdir('results');
end

save(fullfile('results', 'vo_active_results.mat'), ...
    'T_est_all', 'active_size', 'active_corr_count', 'global_fallback_count', ...
    'inlier_count', 'reproj_mean', 'reproj_init_mean', 'added_points', ...
    'map_size', 'frame_status', 'map');

fprintf('Saved active-cloud VO results to results/vo_active_results.mat\n');
fprintf('=== ACTIVE-CLOUD PIPELINE COMPLETED ===\n');
