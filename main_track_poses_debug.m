clear; close all; clc;
warning("off","all");
addpath(genpath("src"));

data_dir = "data";
files = list_measurements(data_dir);

cam = load_camera(fullfile(data_dir, "camera.dat"));
K = cam.K;
T_rc = cam.T_cr;  % robot <- camera

load("init_state_gt.mat"); % state.P (3xN), state.P_app (10xN)

N = numel(files);

% Salviamo pose camera nel mondo (T_wc) come cell array
poses_wc = cell(N,1);

% Mettiamo pose per frame0 e frame1 come quelle dello stato init
poses_wc{1} = state.poses{1};
poses_wc{2} = state.poses{2};

% Salviamo anche una traiettoria (x,y,theta) della camera per plot
traj_est = zeros(N,3);
traj_gt  = zeros(N,3);

% helper per estrarre yaw da SE3 (assumiamo rotazione intorno a z)
yaw_from_T = @(T) atan2(T(2,1), T(1,1));

% Carica meas0/meas1 per GT camera (per confronto)
meas0 = load_measurement(files{1});
meas1 = load_measurement(files{2});
T0_wr_gt = v2t_se3_planar(meas0.gt_pose); T0_wc_gt = T0_wr_gt * T_rc;
T1_wr_gt = v2t_se3_planar(meas1.gt_pose); T1_wc_gt = T1_wr_gt * T_rc;

traj_est(1,:) = [poses_wc{1}(1,4), poses_wc{1}(2,4), yaw_from_T(poses_wc{1})];
traj_est(2,:) = [poses_wc{2}(1,4), poses_wc{2}(2,4), yaw_from_T(poses_wc{2})];

traj_gt(1,:)  = [T0_wc_gt(1,4), T0_wc_gt(2,4), yaw_from_T(T0_wc_gt)];
traj_gt(2,:)  = [T1_wc_gt(1,4), T1_wc_gt(2,4), yaw_from_T(T1_wc_gt)];

th = 0.5;
max_iters = 10;

for i = 3:N
  meas = load_measurement(files{i});

  % --- association frame -> map
  D = appearance_distance(meas.app, state.P_app'); % Nmeas x Nmap
  [best, idx_map] = min(D, [], 2);
  sel = find(best < th);

  P_w = state.P(:, idx_map(sel));   % 3xM
  uv  = meas.uv(sel, :)';           % 2xM

  % --- init guess from odometry (robot -> camera)
  T_wr_guess = v2t_se3_planar(meas.odom_pose);
  T_wc_guess = T_wr_guess * T_rc;
  T_cw_guess = inv(T_wc_guess);

  % --- filter Z>0 in guess
  Pc = T_cw_guess * [P_w; ones(1, columns(P_w))];
  good = find(Pc(3,:) > 0.1);
  P_w = P_w(:, good);
  uv  = uv(:, good);

  if columns(P_w) < 6
    warning("Frame %d: troppo poche corrispondenze (%d). Salto.", i-1, columns(P_w));
    poses_wc{i} = T_wc_guess;
  else
    [T_cw_est, info] = solve_pose_projective_icp(K, T_cw_guess, P_w, uv, max_iters);
    poses_wc{i} = inv(T_cw_est);
  endif

  % --- store traj (est)
  traj_est(i,:) = [poses_wc{i}(1,4), poses_wc{i}(2,4), yaw_from_T(poses_wc{i})];

  % --- GT camera (solo debug confronto)
  T_wr_gt = v2t_se3_planar(meas.gt_pose);
  T_wc_gt = T_wr_gt * T_rc;
  traj_gt(i,:) = [T_wc_gt(1,4), T_wc_gt(2,4), yaw_from_T(T_wc_gt)];

  if mod(i,10)==0 || i==N
    printf("Frame %d/%d | used corr: %d\n", i-1, N-1, columns(P_w));
  endif
endfor

save("-binary", "tracked_debug.mat", "poses_wc", "traj_est", "traj_gt");
printf("Salvato: tracked_debug.mat\n");

% Plot su PNG (no GUI)
graphics_toolkit("gnuplot");
setenv("GNUTERM", "pngcairo");
f = figure("visible","off"); hold on; grid on;
plot(traj_gt(:,1),  traj_gt(:,2),  "-");
plot(traj_est(:,1), traj_est(:,2), "--");
xlabel("x (m)"); ylabel("y (m)");
title("Trajectory (camera): estimated vs GT (debug)");
legend("GT","Estimated");
axis equal;
print(f, "traj_debug.png", "-dpng", "-r150");
close(f);

printf("Salvato: traj_debug.png\n");

