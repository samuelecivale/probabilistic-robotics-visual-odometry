clear;
clc;
close all;

addpath(genpath('src'));

camera = load_camera(fullfile('data', 'camera.dat'));
traj   = load_trajectory(fullfile('data', 'trajectory.dat'));
world  = load_world(fullfile('data', 'world.dat'));

results_file = fullfile('results', 'vo_active_results.mat');
if exist(results_file, 'file') ~= 2
    results_file = fullfile('results', 'vo_incremental_results.mat');
end

load(results_file);

fprintf('=== VISUAL ODOMETRY EVALUATION ===\n');
fprintf('Loaded results from %s\n', results_file);

T_wc_gt = compute_gt_camera_poses(camera, traj);
traj_eval = evaluate_relative_trajectory(T_est_all, T_wc_gt);

fprintf('\n[Trajectory evaluation]\n');
fprintf('Valid relative pairs: %d / %d\n', sum(traj_eval.valid_pair), numel(traj_eval.valid_pair));
fprintf('Mean rotation trace error: %.6f\n', traj_eval.mean_rot_trace_err);
fprintf('Median rotation trace error: %.6f\n', traj_eval.median_rot_trace_err);
fprintf('Mean scale ratio: %.6f\n', traj_eval.mean_scale_ratio);
fprintf('Median scale ratio: %.6f\n', traj_eval.median_scale_ratio);
fprintf('Std scale ratio: %.6f\n', traj_eval.std_scale_ratio);

scale_correction = 1 / traj_eval.median_scale_ratio;
fprintf('Scale correction used: %.6f\n', scale_correction);

n = size(T_est_all, 3);
gt_pos = NaN(n, 3);
est_pos = NaN(n, 3);

T_wc0_gt = T_wc_gt(:, :, 1);
T_c0w_gt = inv(T_wc0_gt);

for k = 1:n
    T_c0ck_gt = T_c0w_gt * T_wc_gt(:, :, k);
    gt_pos(k, :) = T_c0ck_gt(1:3, 4).';

    T_ckc0_est = T_est_all(:, :, k);
    if ~any(isnan(T_ckc0_est(:)))
        T_c0ck_est = inv(T_ckc0_est);
        est_pos(k, :) = (scale_correction * T_c0ck_est(1:3, 4)).';
    end
end

valid_abs = all(~isnan(gt_pos), 2) & all(~isnan(est_pos), 2);
if any(valid_abs)
    pos_err = vecnorm((est_pos(valid_abs, :) - gt_pos(valid_abs, :)).', 2, 1);
    rmse_pos = sqrt(mean(pos_err.^2));
else
    pos_err = [];
    rmse_pos = NaN;
end

fprintf('RMSE position: %.6f m\n', rmse_pos);

map_eval = evaluate_map_exact_descriptors(map, world, T_wc_gt(:, :, 1), scale_correction, 1e-10);

fprintf('\n[Map evaluation]\n');
fprintf('Matched map landmarks: %d\n', map_eval.num_matches);
fprintf('Map RMSE: %.6f m\n', map_eval.rmse);
fprintf('Mean map error: %.6f m\n', map_eval.mean_err);
fprintf('Median map error: %.6f m\n', map_eval.median_err);

if exist('results', 'dir') ~= 7
    mkdir('results');
end

save(fullfile('results', 'evaluation_results.mat'), ...
    'traj_eval', 'map_eval', 'gt_pos', 'est_pos', 'rmse_pos', 'scale_correction');

fprintf('\nSaved evaluation data to results/evaluation_results.mat\n');
fprintf('=== EVALUATION COMPLETED ===\n');
