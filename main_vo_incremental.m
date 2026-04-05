clear;
clc;
close all;

addpath(genpath('src'));

camera = load_camera(fullfile('data', 'camera.dat'));
traj   = load_trajectory(fullfile('data', 'trajectory.dat'));

max_desc_dist = 1e-8;
max_reproj_err_add = 2.0;
max_pose_err_for_map_add = 5.0;

refine_params = struct();
refine_params.max_iters = 10;
refine_params.lambda = 1e-6;
refine_params.huber_delta = 5.0;
refine_params.min_depth = 1e-8;
refine_params.min_points = 6;
refine_params.step_tol = 1e-8;

num_frames = numel(traj.pose_id);
measurements = load_measurement_sequence('data', num_frames);

meas0 = measurements{1};
meas1 = measurements{2};

[map, init] = build_initial_map_struct(camera, meas0, meas1);

fprintf('=== INCREMENTAL VISUAL ODOMETRY ===\n');
fprintf('Initial map points: %d\n', size(map.points, 1));
fprintf('Descriptor threshold: %.1e\n', max_desc_dist);

T_est_all = NaN(4, 4, num_frames);
assoc_count = zeros(num_frames, 1);
reproj_mean = NaN(num_frames, 1);
reproj_init_mean = NaN(num_frames, 1);
map_size = zeros(num_frames, 1);
added_points = zeros(num_frames, 1);
frame_status = cell(num_frames, 1);

T_est_all(:, :, 1) = eye(4);
assoc_count(1) = size(map.points, 1);
map_size(1) = size(map.points, 1);
frame_status{1} = 'init_identity';

T_10 = eye(4);
T_10(1:3,1:3) = init.R01;
T_10(1:3,4) = init.t01(:);
T_est_all(:, :, 2) = T_10;
assoc_count(2) = size(map.points, 1);
map_size(2) = size(map.points, 1);
frame_status{2} = 'init_two_view';

fprintf('frame %03d | status=%s | map=%d\n', 0, frame_status{1}, map_size(1));
fprintf('frame %03d | status=%s | map=%d\n', 1, frame_status{2}, map_size(2));

for k = 3:num_frames
    meas_prev = measurements{k-1};
    meas_curr = measurements{k};

    assoc = match_by_appearance_mutual_threshold(map.desc, meas_curr.appearance, max_desc_dist);
    assoc_count(k) = numel(assoc.idx1);

    if assoc_count(k) < 6
        frame_status{k} = 'too_few_matches';
        map_size(k) = size(map.points, 1);

        if k <= 10 || mod(k-1, 5) == 0 || k == num_frames
            fprintf('frame %03d | status=%s | matches=%d | map=%d\n', ...
                k-1, frame_status{k}, assoc_count(k), map_size(k));
        end
        continue;
    end

    Pw_corr = map.points(assoc.idx1, :);
    uv_corr = meas_curr.uv(assoc.idx2, :);
    xn_corr = pixels_to_normalized(camera.K, uv_corr);

    pose_init = estimate_pose_dlt(Pw_corr, xn_corr);
    T_init = pose_to_matrix(pose_init.R, pose_init.t);

    [~, ~, err_init, valid_init] = reprojection_errors_pose(camera.K, T_init, Pw_corr, uv_corr, 1e-8);
    if any(valid_init)
        reproj_init_mean(k) = mean(err_init(valid_init));
    end

    refine_result = refine_pose_projective_icp(camera.K, T_init, Pw_corr, uv_corr, refine_params);
    T_curr = refine_result.T;

    T_est_all(:, :, k) = T_curr;
    frame_status{k} = 'ok';

    if refine_result.success
        reproj_mean(k) = refine_result.mean_err;
    else
        [~, ~, err_curr, valid_curr] = reprojection_errors_pose(camera.K, T_curr, Pw_corr, uv_corr, 1e-8);
        if any(valid_curr)
            reproj_mean(k) = mean(err_curr(valid_curr));
        end
    end

    T_prev = T_est_all(:, :, k-1);
    if all(~isnan(T_prev(:))) && ~isnan(reproj_mean(k)) && reproj_mean(k) < max_pose_err_for_map_add
        [map, stats] = add_new_landmarks_from_frame_pair(map, meas_prev, meas_curr, ...
            T_prev, T_curr, camera, max_desc_dist, max_reproj_err_add, assoc.idx2);
        added_points(k) = stats.added_points;
    end

    map_size(k) = size(map.points, 1);

    if k <= 10 || mod(k-1, 5) == 0 || k == num_frames
        fprintf('frame %03d | status=%s | matches=%d | init_reproj=%.6f px | refined_reproj=%.6f px | added=%d | map=%d\n', ...
            k-1, frame_status{k}, assoc_count(k), reproj_init_mean(k), reproj_mean(k), added_points(k), map_size(k));
    end
end

num_ok = 0;
for k = 1:num_frames
    if strcmp(frame_status{k}, 'ok') || strcmp(frame_status{k}, 'init_identity') || strcmp(frame_status{k}, 'init_two_view')
        num_ok = num_ok + 1;
    end
end

fprintf('\nProcessed frames: %d\n', num_frames);
fprintf('Successful frames: %d\n', num_ok);
fprintf('Final map size: %d\n', size(map.points, 1));
fprintf('Average associations on tracked frames: %.2f\n', mean(assoc_count(assoc_count > 0)));

valid_reproj = ~isnan(reproj_mean);
if any(valid_reproj)
    fprintf('Average refined reprojection error on valid frames: %.6f px\n', mean(reproj_mean(valid_reproj)));
end

valid_reproj_init = ~isnan(reproj_init_mean);
if any(valid_reproj_init)
    fprintf('Average initial reprojection error on valid frames: %.6f px\n', mean(reproj_init_mean(valid_reproj_init)));
end

if exist('results', 'dir') ~= 7
    mkdir('results');
end

save(fullfile('results', 'vo_incremental_results.mat'), ...
    'T_est_all', 'assoc_count', 'reproj_mean', 'reproj_init_mean', ...
    'map_size', 'added_points', 'frame_status', 'map');

fprintf('Saved incremental VO results to results/vo_incremental_results.mat\n');
fprintf('=== INCREMENTAL PIPELINE COMPLETED ===\n');
