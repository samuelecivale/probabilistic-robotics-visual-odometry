clear;
clc;
close all;

addpath(genpath('src'));

camera = load_camera(fullfile('data', 'camera.dat'));
traj   = load_trajectory(fullfile('data', 'trajectory.dat'));
meas0  = load_measurement(fullfile('data', 'meas-00000.dat'));
meas1  = load_measurement(fullfile('data', 'meas-00001.dat'));

matches = match_by_appearance_mutual(meas0.appearance, meas1.appearance);

fprintf('=== TWO-VIEW INITIALIZATION ===\n');
fprintf('Mutual matches: %d\n', numel(matches.idx1));

uv0 = meas0.uv(matches.idx1, :);
uv1 = meas1.uv(matches.idx2, :);

x0 = pixels_to_normalized(camera.K, uv0);
x1 = pixels_to_normalized(camera.K, uv1);

E = estimate_essential_eight_point(x0, x1);
candidates = decompose_essential(E);
result = select_essential_pose(candidates, x0, x1);

fprintf('Cheirality inliers: %d / %d\n', result.num_valid, size(x0,1));

R_est = result.R;
t_est = result.t / norm(result.t);

fprintf('\nEstimated relative pose (camera 0 -> camera 1)\n');
disp(R_est);
fprintf('Estimated translation direction:\n');
disp(t_est.');

% GT relative camera motion for debug
T_wr_0 = pose2d_to_se3(traj.gt_pose(1, :));
T_wr_1 = pose2d_to_se3(traj.gt_pose(2, :));

T_wc_0 = T_wr_0 * camera.cam_transform;
T_wc_1 = T_wr_1 * camera.cam_transform;

T_c0w = inv(T_wc_0);
T_c1w = inv(T_wc_1);

T_c1c0_gt = T_c1w * T_wc_0;

R_gt = T_c1c0_gt(1:3,1:3);
t_gt = T_c1c0_gt(1:3,4);
t_gt = t_gt / norm(t_gt);

R_err = R_est' * R_gt;
rot_trace_err = trace(eye(3) - R_err);
trans_dir_dot = dot(t_est, t_gt);

fprintf('\nGT relative pose (for debug only)\n');
fprintf('Rotation trace error: %.6f\n', rot_trace_err);
fprintf('Translation direction dot product: %.6f\n', trans_dir_dot);

Pw_init = result.Pw(result.valid_cheirality, :);

fprintf('Triangulated points after cheirality: %d\n', size(Pw_init,1));

if exist('results', 'dir') ~= 7
    mkdir('results');
end

save(fullfile('results', 'init_state_vo.mat'), ...
    'E', 'R_est', 't_est', 'Pw_init', 'matches', 'result');

fprintf('Saved initialization state to results/init_state_vo.mat\n');
fprintf('=== INITIALIZATION COMPLETED ===\n');
