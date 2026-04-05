clear;
clc;
close all;

addpath(genpath('src'));

camera = load_camera(fullfile('data', 'camera.dat'));
traj   = load_trajectory(fullfile('data', 'trajectory.dat'));
world  = load_world(fullfile('data', 'world.dat'));
meas0  = load_measurement(fullfile('data', 'meas-00000.dat'));
meas1  = load_measurement(fullfile('data', 'meas-00001.dat'));

fprintf('=== LOADERS TEST ===\n');

fprintf('\n[camera]\n');
fprintf('K size: %d x %d\n', size(camera.K,1), size(camera.K,2));
fprintf('cam_transform size: %d x %d\n', size(camera.cam_transform,1), size(camera.cam_transform,2));
fprintf('z_near: %.3f\n', camera.z_near);
fprintf('z_far: %.3f\n', camera.z_far);
fprintf('width: %.0f\n', camera.width);
fprintf('height: %.0f\n', camera.height);

fprintf('\n[trajectory]\n');
fprintf('num poses: %d\n', numel(traj.pose_id));
fprintf('odom_pose size: %d x %d\n', size(traj.odom_pose,1), size(traj.odom_pose,2));
fprintf('gt_pose size: %d x %d\n', size(traj.gt_pose,1), size(traj.gt_pose,2));

fprintf('\n[world]\n');
fprintf('num landmarks: %d\n', numel(world.id));
fprintf('position size: %d x %d\n', size(world.position,1), size(world.position,2));
fprintf('appearance size: %d x %d\n', size(world.appearance,1), size(world.appearance,2));

fprintf('\n[measurement 0]\n');
fprintf('seq: %d\n', meas0.seq);
fprintf('gt_pose size: %d x %d\n', size(meas0.gt_pose,1), size(meas0.gt_pose,2));
fprintf('odom_pose size: %d x %d\n', size(meas0.odom_pose,1), size(meas0.odom_pose,2));
fprintf('num observed points: %d\n', numel(meas0.point_id));
fprintf('uv size: %d x %d\n', size(meas0.uv,1), size(meas0.uv,2));
fprintf('appearance size: %d x %d\n', size(meas0.appearance,1), size(meas0.appearance,2));

fprintf('\n[measurement 1]\n');
fprintf('seq: %d\n', meas1.seq);
fprintf('num observed points: %d\n', numel(meas1.point_id));

assert(all(size(camera.K) == [3, 3]));
assert(all(size(camera.cam_transform) == [4, 4]));
assert(size(traj.odom_pose, 2) == 3);
assert(size(traj.gt_pose, 2) == 3);
assert(size(world.position, 2) == 3);
assert(size(world.appearance, 2) == 10);
assert(size(meas0.uv, 2) == 2);
assert(size(meas0.appearance, 2) == 10);
assert(size(meas1.uv, 2) == 2);
assert(size(meas1.appearance, 2) == 10);

fprintf('\nAll loader checks passed.\n');
