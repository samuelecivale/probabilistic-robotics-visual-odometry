clear; close all; clc;
warning("off","all");
addpath(genpath("src"));

data_dir = "data";
files = list_measurements(data_dir);

% --- camera intrinsics + transform
cam = load_camera(fullfile(data_dir, "camera.dat"));
K = cam.K;
T_rc = cam.T_cr;  % INTERPRETATO COME robot <- camera

% --- load initial map (built from frame0-1 using GT, now with correct T_rc)
load("init_state_gt.mat");  % struct 'state': P (3xN), P_app (10xN), poses camera

% --- load frame2
meas2 = load_measurement(files{3});  % seq 2

% --- associate frame2 measurements to map via appearance
D = appearance_distance(meas2.app, state.P_app');    % Nmeas x Nmap
[best, idx_map] = min(D, [], 2);

th = 0.5;
sel = find(best < th);

printf("Frame2: %d meas | Map: %d pts | Assoc (th=%.2f): %d\n", ...
       rows(meas2.app), columns(state.P), th, numel(sel));

% build 2D-3D sets
P_w = state.P(:, idx_map(sel));      % 3xN
uv  = meas2.uv(sel, :)';             % 2xN

% --- init pose guess from odometry (robot pose) -> camera pose
T2_wr_guess = v2t_se3_planar(meas2.odom_pose);   % world <- robot
T2_wc_guess = T2_wr_guess * T_rc;                % world <- camera  (T_wc = T_wr * T_rc)
T2_cw_guess = inv(T2_wc_guess);                  % camera <- world

% filter correspondences with positive depth in initial guess
Pc = T2_cw_guess * [P_w; ones(1, columns(P_w))];
good = find(Pc(3,:) > 0.1);
P_w = P_w(:, good);
uv  = uv(:, good);

printf("Dopo filtro Z>0 nella posa iniziale: %d corrispondenze\n", columns(P_w));

% --- solve pose (projective ICP / PnP)
[T_cw_est, info] = solve_pose_projective_icp(K, T2_cw_guess, P_w, uv, 10);
T_wc_est = inv(T_cw_est);

printf("Projective ICP iters: %d | final RMSE reproj: %.3f px\n", info.iters, info.final_rmse);

% --- debug vs GT (convert robot GT -> camera GT)
T2_wr_gt = v2t_se3_planar(meas2.gt_pose);
T2_wc_gt = T2_wr_gt * T_rc;

dp = T2_wc_gt(1:3,4) - T_wc_est(1:3,4);
printf("Translation error (camera in world): %.4f m\n", norm(dp));

