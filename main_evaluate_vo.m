clear;
clc;
close all;

addpath(genpath('src'));

camera = load_camera(fullfile('data', 'camera.dat'));
traj   = load_trajectory(fullfile('data', 'trajectory.dat'));
world  = load_world(fullfile('data', 'world.dat'));

load(fullfile('results', 'vo_incremental_results.mat')); % loads T_est_all, map, ...

fprintf('=== VISUAL ODOMETRY EVALUATION ===\n');

T_wc_gt = compute_gt_camera_poses(camera, traj);

traj_eval = evaluate_relative_trajectory(T_est_all, T_wc_gt);

fprintf('\n[Trajectory evaluation]\n');
fprintf('Valid relative pairs: %d / %d\n', sum(traj_eval.valid_pair), numel(traj_eval.valid_pair));
fprintf('Mean rotation trace error: %.6f\n', traj_eval.mean_rot_trace_err);
fprintf('Median rotation trace error: %.6f\n', traj_eval.median_rot_trace_err);
fprintf('Mean scale ratio: %.6f\n', traj_eval.mean_scale_ratio);
fprintf('Median scale ratio: %.6f\n', traj_eval.median_scale_ratio);
fprintf('Std scale ratio: %.6f\n', traj_eval.std_scale_ratio);

scale_ratio_used = traj_eval.median_scale_ratio;

map_eval = evaluate_map_from_descriptors(map, world, T_wc_gt(:, :, 1), scale_ratio_used, 1e-8);

fprintf('\n[Map evaluation]\n');
fprintf('Matched map landmarks: %d\n', map_eval.num_matches);
fprintf('Scale used for map: %.6f\n', scale_ratio_used);
fprintf('Map RMSE: %.6f m\n', map_eval.rmse);
fprintf('Mean map error: %.6f m\n', map_eval.mean_err);
fprintf('Median map error: %.6f m\n', map_eval.median_err);

% Camera trajectories in camera-0 frame for plotting
n = size(T_est_all, 3);
gt_xy = NaN(n, 2);
est_xy = NaN(n, 2);

T_wc0_gt = T_wc_gt(:, :, 1);
T_c0w_gt = inv(T_wc0_gt);

for k = 1:n
    % GT camera pose in c0 frame
    T_c0ck_gt = T_c0w_gt * T_wc_gt(:, :, k);
    gt_xy(k, :) = T_c0ck_gt(1:2, 4).';

    % Estimated camera pose in c0 frame
    T_ckc0_est = T_est_all(:, :, k);
    if ~any(isnan(T_ckc0_est(:)))
        T_c0ck_est = inv(T_ckc0_est);
        est_xy(k, :) = T_c0ck_est(1:2, 4).';
    end
end

if exist('results', 'dir') ~= 7
    mkdir('results');
end

save(fullfile('results', 'evaluation_results.mat'), 'traj_eval', 'map_eval', 'gt_xy', 'est_xy', 'scale_ratio_used');

% Try to save plots, but do not fail if plotting backend complains
try
    graphics_toolkit('gnuplot');
catch
end

try
    f1 = figure('visible', 'off');
    plot(find(traj_eval.valid_pair), traj_eval.rot_trace_err(traj_eval.valid_pair), 'LineWidth', 1.5);
    grid on;
    xlabel('relative motion index');
    ylabel('trace(I - R_{err})');
    title('Relative rotation error');
    print(f1, fullfile('results', 'rot_error.png'), '-dpng');
    close(f1);
    fprintf('Saved results/rot_error.png\n');
catch err
    fprintf('Plot rot_error skipped: %s\n', err.message);
end

try
    f2 = figure('visible', 'off');
    plot(find(traj_eval.valid_scale), traj_eval.scale_ratio(traj_eval.valid_scale), 'LineWidth', 1.5);
    grid on;
    xlabel('relative motion index');
    ylabel('||t_est|| / ||t_gt||');
    title('Scale ratio per relative motion');
    print(f2, fullfile('results', 'scale_ratio.png'), '-dpng');
    close(f2);
    fprintf('Saved results/scale_ratio.png\n');
catch err
    fprintf('Plot scale_ratio skipped: %s\n', err.message);
end

try
    f3 = figure('visible', 'off');
    plot(gt_xy(:,1), gt_xy(:,2), 'LineWidth', 1.5);
    hold on;
    plot(est_xy(:,1), est_xy(:,2), 'LineWidth', 1.5);
    grid on;
    axis equal;
    xlabel('x');
    ylabel('y');
    title('Camera trajectory in frame c0');
    legend('GT', 'Estimated');
    print(f3, fullfile('results', 'trajectory_xy.png'), '-dpng');
    close(f3);
    fprintf('Saved results/trajectory_xy.png\n');
catch err
    fprintf('Plot trajectory_xy skipped: %s\n', err.message);
end

fprintf('\nSaved evaluation data to results/evaluation_results.mat\n');
fprintf('=== EVALUATION COMPLETED ===\n');
