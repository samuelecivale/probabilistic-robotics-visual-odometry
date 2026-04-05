clear;
clc;
close all;

addpath(genpath('src'));

camera = load_camera(fullfile('data', 'camera.dat'));
traj   = load_trajectory(fullfile('data', 'trajectory.dat'));
world  = load_world(fullfile('data', 'world.dat'));
meas0  = load_measurement(fullfile('data', 'meas-00000.dat'));

fprintf('=== PROJECTION SANITY CHECK ===\n');

% Ground-truth robot pose for frame 0
robot_pose = traj.gt_pose(1, :);
T_wr = pose2d_to_se3(robot_pose);   % robot pose in world

% Build a map from landmark id to row index in world
max_id = max(world.id);
id_to_row = zeros(max_id + 1, 1);

for i = 1:numel(world.id)
    id_to_row(world.id(i) + 1) = i;
end

obs_ids = meas0.point_id;
obs_uv  = meas0.uv;

Pw = zeros(numel(obs_ids), 3);
valid_ids_mask = true(numel(obs_ids), 1);

for i = 1:numel(obs_ids)
    wid = obs_ids(i);
    idx = wid + 1;

    if idx < 1 || idx > numel(id_to_row) || id_to_row(idx) == 0
        valid_ids_mask(i) = false;
    else
        Pw(i, :) = world.position(id_to_row(idx), :);
    end
end

Pw = Pw(valid_ids_mask, :);
obs_uv = obs_uv(valid_ids_mask, :);

fprintf('Observed landmarks used for check: %d\n', size(Pw, 1));

% Interpretation A:
% camera.cam_transform is T_rc (camera pose w.r.t. robot, mapped as robot <- camera)
T_rc_A = camera.cam_transform;
T_wc_A = T_wr * T_rc_A;
T_cw_A = inv(T_wc_A);

[uv_A, valid_A] = project_points(camera.K, T_cw_A, Pw, camera.width, camera.height, camera.z_near, camera.z_far);

% Interpretation B:
% camera.cam_transform should be inverted before composing
T_rc_B = inv(camera.cam_transform);
T_wc_B = T_wr * T_rc_B;
T_cw_B = inv(T_wc_B);

[uv_B, valid_B] = project_points(camera.K, T_cw_B, Pw, camera.width, camera.height, camera.z_near, camera.z_far);

% Reprojection errors only on valid projected points
err_A = vecnorm((uv_A(valid_A, :) - obs_uv(valid_A, :)).', 2, 1);
err_B = vecnorm((uv_B(valid_B, :) - obs_uv(valid_B, :)).', 2, 1);

fprintf('\nInterpretation A:\n');
fprintf('  valid projections: %d / %d\n', sum(valid_A), numel(valid_A));
if ~isempty(err_A)
    fprintf('  mean reprojection error: %.4f px\n', mean(err_A));
    fprintf('  median reprojection error: %.4f px\n', median(err_A));
else
    fprintf('  no valid projected points\n');
end

fprintf('\nInterpretation B:\n');
fprintf('  valid projections: %d / %d\n', sum(valid_B), numel(valid_B));
if ~isempty(err_B)
    fprintf('  mean reprojection error: %.4f px\n', mean(err_B));
    fprintf('  median reprojection error: %.4f px\n', median(err_B));
else
    fprintf('  no valid projected points\n');
end

if ~isempty(err_A) && ~isempty(err_B)
    if median(err_A) < median(err_B)
        fprintf('\nBest interpretation: A (use camera.cam_transform directly)\n');
    else
        fprintf('\nBest interpretation: B (invert camera.cam_transform before composing)\n');
    end
end

% Optional quick plot for best interpretation
figure;
hold on;
axis ij;
xlim([0 camera.width]);
ylim([0 camera.height]);
title('GT projection sanity check on frame 0');
xlabel('u');
ylabel('v');

plot(obs_uv(:,1), obs_uv(:,2), 'bo', 'markersize', 4);

if ~isempty(err_A) && ~isempty(err_B) && median(err_A) < median(err_B)
    plot(uv_A(valid_A,1), uv_A(valid_A,2), 'r+');
    legend('observed', 'projected (best)');
else
    plot(uv_B(valid_B,1), uv_B(valid_B,2), 'r+');
    legend('observed', 'projected (best)');
end

grid on;

mkdir('results');
saveas(gcf, fullfile('results', 'projection_sanity_check_frame0.png'));

fprintf('\nSaved plot to results/projection_sanity_check_frame0.png\n');
