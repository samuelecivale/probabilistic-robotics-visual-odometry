clear;
clc;
close all;

addpath(genpath('src'));

camera = load_camera(fullfile('data', 'camera.dat'));
traj   = load_trajectory(fullfile('data', 'trajectory.dat'));
meas0  = load_measurement(fullfile('data', 'meas-00000.dat'));
meas1  = load_measurement(fullfile('data', 'meas-00001.dat'));
meas2  = load_measurement(fullfile('data', 'meas-00002.dat'));

fprintf('=== FRAME-TO-MAP POSE ESTIMATION TEST ===\n');

% --- Step 1: initialize map from frame 0 and 1
matches01 = match_by_appearance_mutual(meas0.appearance, meas1.appearance);

uv0 = meas0.uv(matches01.idx1, :);
uv1 = meas1.uv(matches01.idx2, :);

x0 = pixels_to_normalized(camera.K, uv0);
x1 = pixels_to_normalized(camera.K, uv1);

E = estimate_essential_eight_point(x0, x1);
candidates = decompose_essential(E);
init_result = select_essential_pose(candidates, x0, x1);

Pw_map = init_result.Pw(init_result.valid_cheirality, :);
map_desc = meas0.appearance(matches01.idx1(init_result.valid_cheirality), :);
map_ids  = meas0.point_id(matches01.idx1(init_result.valid_cheirality));

fprintf('Initialized map points: %d\n', size(Pw_map, 1));

% --- Step 2: associate frame 2 observations to map via appearance
assoc = associate_frame_to_map_by_appearance(map_desc, meas2.appearance);

fprintf('Frame 2 associations: %d\n', numel(assoc.map_idx));

assoc_correct = (map_ids(assoc.map_idx) == meas2.point_id(assoc.frame_idx));
fprintf('Association accuracy (debug with GT ids): %d / %d = %.4f\n', ...
    sum(assoc_correct), numel(assoc_correct), mean(assoc_correct));

Pw_corr = Pw_map(assoc.map_idx, :);
uv2_corr = meas2.uv(assoc.frame_idx, :);
x2_corr = pixels_to_normalized(camera.K, uv2_corr);

% --- Step 3: estimate frame 2 pose wrt initial map frame (frame 0 camera frame)
pose2 = estimate_pose_dlt(Pw_corr, x2_corr);

fprintf('\nEstimated pose for frame 2 (camera wrt init map)\n');
disp(pose2.R);
fprintf('Estimated translation:\n');
disp(pose2.t.');

T_c2c0_est = pose_to_matrix(pose2.R, pose2.t);

% --- Step 4: reprojection error
[uv2_proj, valid2_proj] = project_points(camera.K, T_c2c0_est, Pw_corr, ...
    camera.width, camera.height, camera.z_near, 1e9);

reproj_err = vecnorm((uv2_proj(valid2_proj, :) - uv2_corr(valid2_proj, :)).', 2, 1);

fprintf('\nReprojection valid points: %d / %d\n', sum(valid2_proj), numel(valid2_proj));
fprintf('Mean reprojection error: %.6f px\n', mean(reproj_err));
fprintf('Median reprojection error: %.6f px\n', median(reproj_err));

% --- Step 5: compare with GT relative pose (debug only)
T_wr_0 = pose2d_to_se3(traj.gt_pose(1, :));
T_wr_2 = pose2d_to_se3(traj.gt_pose(3, :));

T_wc_0_gt = T_wr_0 * camera.cam_transform;
T_wc_2_gt = T_wr_2 * camera.cam_transform;

T_c2c0_gt = inv(T_wc_2_gt) * T_wc_0_gt;

R_gt = T_c2c0_gt(1:3, 1:3);
t_gt = T_c2c0_gt(1:3, 4);
t_gt = t_gt / norm(t_gt);

t_est_dir = pose2.t / norm(pose2.t);

R_err = pose2.R' * R_gt;
rot_trace_err = trace(eye(3) - R_err);
trans_dir_dot = dot(t_est_dir, t_gt);

fprintf('\nGT comparison (debug only)\n');
fprintf('Rotation trace error: %.6f\n', rot_trace_err);
fprintf('Translation direction dot product: %.6f\n', trans_dir_dot);

if exist('results', 'dir') ~= 7
    mkdir('results');
end

save(fullfile('results', 'pose_test_frame2.mat'), ...
    'pose2', 'T_c2c0_est', 'Pw_map', 'map_desc', 'map_ids', 'assoc');

fprintf('Saved frame 2 pose test to results/pose_test_frame2.mat\n');
fprintf('=== POSE ESTIMATION TEST COMPLETED ===\n');
