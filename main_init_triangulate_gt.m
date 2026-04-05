clear;
clc;
close all;

addpath(genpath('src'));

camera = load_camera(fullfile('data', 'camera.dat'));
traj   = load_trajectory(fullfile('data', 'trajectory.dat'));
world  = load_world(fullfile('data', 'world.dat'));
meas0  = load_measurement(fullfile('data', 'meas-00000.dat'));
meas1  = load_measurement(fullfile('data', 'meas-00001.dat'));

matches = match_by_appearance_mutual(meas0.appearance, meas1.appearance);

fprintf('=== INITIAL TRIANGULATION WITH GT POSES ===\n');
fprintf('Mutual matches: %d\n', numel(matches.idx1));

% GT robot poses
T_wr_0 = pose2d_to_se3(traj.gt_pose(1, :));
T_wr_1 = pose2d_to_se3(traj.gt_pose(2, :));

% Camera pose in world
T_wc_0 = T_wr_0 * camera.cam_transform;
T_wc_1 = T_wr_1 * camera.cam_transform;

% World to camera
T_cw_0 = inv(T_wc_0);
T_cw_1 = inv(T_wc_1);

% Projection matrices
P0 = camera.K * T_cw_0(1:3, :);
P1 = camera.K * T_cw_1(1:3, :);

UV0 = meas0.uv(matches.idx1, :);
UV1 = meas1.uv(matches.idx2, :);
landmark_ids0 = meas0.point_id(matches.idx1);
landmark_ids1 = meas1.point_id(matches.idx2);

assert(all(landmark_ids0 == landmark_ids1), 'Mutual matches should refer to same landmark id');

Pw_est = triangulate_points_linear(P0, P1, UV0, UV1);

% Cheirality check
n = size(Pw_est, 1);
valid_cheirality = false(n, 1);

for i = 1:n
    Xh = [Pw_est(i, :) 1].';

    pc0 = T_cw_0 * Xh;
    pc1 = T_cw_1 * Xh;

    z0 = pc0(3);
    z1 = pc1(3);

    valid_cheirality(i) = (z0 > camera.z_near) && (z0 < camera.z_far) && ...
                          (z1 > camera.z_near) && (z1 < camera.z_far);
end

fprintf('Cheirality-valid points: %d / %d\n', sum(valid_cheirality), n);

Pw_est_valid = Pw_est(valid_cheirality, :);
valid_ids = landmark_ids0(valid_cheirality);

% Recover GT world points by landmark id
max_id = max(world.id);
id_to_row = zeros(max_id + 1, 1);

for i = 1:numel(world.id)
    id_to_row(world.id(i) + 1) = i;
end

Pw_gt = zeros(numel(valid_ids), 3);
for i = 1:numel(valid_ids)
    Pw_gt(i, :) = world.position(id_to_row(valid_ids(i) + 1), :);
end

errors = vecnorm((Pw_est_valid - Pw_gt).', 2, 1);
rmse = sqrt(mean(errors .^ 2));

fprintf('Triangulated valid points: %d\n', size(Pw_est_valid, 1));
fprintf('Map RMSE against GT: %.6f m\n', rmse);
fprintf('Mean point error: %.6f m\n', mean(errors));
fprintf('Median point error: %.6f m\n', median(errors));

% Reprojection check on frame 0 and 1
[uv0_proj, valid0_proj] = project_points(camera.K, T_cw_0, Pw_est_valid, ...
    camera.width, camera.height, camera.z_near, camera.z_far);
[uv1_proj, valid1_proj] = project_points(camera.K, T_cw_1, Pw_est_valid, ...
    camera.width, camera.height, camera.z_near, camera.z_far);

reproj_mask = valid0_proj & valid1_proj;

err_reproj_0 = vecnorm((uv0_proj(reproj_mask, :) - UV0(valid_cheirality(reproj_mask), :)).', 2, 1);
err_reproj_1 = vecnorm((uv1_proj(reproj_mask, :) - UV1(valid_cheirality(reproj_mask), :)).', 2, 1);

fprintf('Reprojection valid points: %d\n', sum(reproj_mask));
if ~isempty(err_reproj_0)
    fprintf('Frame 0 mean reprojection error: %.6f px\n', mean(err_reproj_0));
    fprintf('Frame 1 mean reprojection error: %.6f px\n', mean(err_reproj_1));
end

if exist('results', 'dir') ~= 7
    mkdir('results');
end

save(fullfile('results', 'init_state_gt.mat'), 'Pw_est', 'Pw_est_valid', 'valid_ids', ...
     'matches', 'T_wr_0', 'T_wr_1', 'T_wc_0', 'T_wc_1', 'T_cw_0', 'T_cw_1');

fprintf('Saved initialization state to results/init_state_gt.mat\n');
fprintf('=== TRIANGULATION TEST COMPLETED ===\n');
