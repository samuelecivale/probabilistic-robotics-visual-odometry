clear;
clc;
close all;

addpath(genpath('src'));

camera = load_camera(fullfile('data', 'camera.dat'));
traj   = load_trajectory(fullfile('data', 'trajectory.dat'));

meas0 = load_measurement(fullfile('data', 'meas-00000.dat'));
meas1 = load_measurement(fullfile('data', 'meas-00001.dat'));

init = build_initial_map_from_two_view(camera, meas0, meas1);

fprintf('=== FULL VISUAL ODOMETRY PIPELINE ===\n');
fprintf('Initial map points: %d\n', init.num_map_points);

num_frames = numel(traj.pose_id);

T_est_all = NaN(4, 4, num_frames);
assoc_count = zeros(num_frames, 1);
assoc_accuracy = NaN(num_frames, 1);
reproj_mean = NaN(num_frames, 1);
reproj_median = NaN(num_frames, 1);
frame_status = cell(num_frames, 1);

% Frame 0: identity in map frame
T_est_all(:, :, 1) = eye(4);
assoc_count(1) = init.num_map_points;
frame_status{1} = 'init_identity';

% Frame 1: from two-view initialization
T_10 = eye(4);
T_10(1:3,1:3) = init.R01;
T_10(1:3,4) = init.t01(:);
T_est_all(:, :, 2) = T_10;
assoc_count(2) = init.num_map_points;
frame_status{2} = 'init_two_view';

fprintf('frame %03d | status=%s | map pts=%d\n', 0, frame_status{1}, assoc_count(1));
fprintf('frame %03d | status=%s | map pts=%d\n', 1, frame_status{2}, assoc_count(2));

for k = 3:num_frames
    meas_file = fullfile('data', sprintf('meas-%05d.dat', k-1));
    meas = load_measurement(meas_file);

    assoc = associate_frame_to_map_by_appearance(init.map_desc, meas.appearance);
    assoc_count(k) = numel(assoc.map_idx);

    if assoc_count(k) < 6
        frame_status{k} = 'too_few_matches';
        fprintf('frame %03d | status=%s | matches=%d\n', k-1, frame_status{k}, assoc_count(k));
        continue;
    end

    Pw_corr = init.Pw_map(assoc.map_idx, :);
    uv_corr = meas.uv(assoc.frame_idx, :);
    xn_corr = pixels_to_normalized(camera.K, uv_corr);

    pose = estimate_pose_dlt(Pw_corr, xn_corr);
    T_est = pose_to_matrix(pose.R, pose.t);

    T_est_all(:, :, k) = T_est;
    frame_status{k} = 'ok';
    [uv_proj, valid_proj] = project_points(camera.K, T_est, Pw_corr, ...
    camera.width, camera.height, camera.z_near, 1e9);

    % Debug only: check association correctness with GT ids
    correct = (init.map_ids(assoc.map_idx) == meas.point_id(assoc.frame_idx));
    assoc_accuracy(k) = mean(correct);

    if any(valid_proj)
        err = vecnorm((uv_proj(valid_proj, :) - uv_corr(valid_proj, :)).', 2, 1);
        reproj_mean(k) = mean(err);
        reproj_median(k) = median(err);

        fprintf('frame %03d | status=%s | matches=%d | assoc_acc=%.4f | reproj_mean=%.6f px\n', ...
            k-1, frame_status{k}, assoc_count(k), assoc_accuracy(k), reproj_mean(k));
    else
        reproj_mean(k) = NaN;
        reproj_median(k) = NaN;
        frame_status{k} = 'ok_no_valid_reprojection';

        fprintf('frame %03d | status=%s | matches=%d | assoc_acc=%.4f | reproj_mean=NaN\n', ...
            k-1, frame_status{k}, assoc_count(k), assoc_accuracy(k));
    end
end

num_ok = 0;
num_dlt = 0;
for k = 1:num_frames
    if strcmp(frame_status{k}, 'ok') || strcmp(frame_status{k}, 'init_identity') || strcmp(frame_status{k}, 'init_two_view')
        num_ok = num_ok + 1;
    end
    if strcmp(frame_status{k}, 'ok')
        num_dlt = num_dlt + 1;
    end
end

fprintf('\nProcessed frames: %d\n', num_frames);
fprintf('Successful frames: %d\n', num_ok);
fprintf('Frames with pose estimates from DLT: %d\n', num_dlt);
fprintf('Average associations on tracked frames: %.2f\n', mean(assoc_count(assoc_count > 0)));

valid_reproj = ~isnan(reproj_mean);
if any(valid_reproj)
    fprintf('Average reprojection error over valid tracked frames: %.6f px\n', mean(reproj_mean(valid_reproj)));
end

if exist('results', 'dir') ~= 7
    mkdir('results');
end

save(fullfile('results', 'vo_full_results.mat'), ...
    'T_est_all', 'assoc_count', 'assoc_accuracy', 'reproj_mean', ...
    'reproj_median', 'frame_status', 'init');

fprintf('Saved full VO results to results/vo_full_results.mat\n');
fprintf('=== FULL PIPELINE COMPLETED ===\n');
