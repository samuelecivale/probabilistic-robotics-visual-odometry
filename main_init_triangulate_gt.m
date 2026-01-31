clear; close all; clc;
warning("off","all");
addpath(genpath("src"));

data_dir = "data";
files = list_measurements(data_dir);

% --- load camera intrinsics + cam_transform
cam = load_camera(fullfile(data_dir, "camera.dat"));
K = cam.K;
T_rc = cam.T_cr;  % ATTENZIONE: lo useremo come robot <- camera

printf("K =\n"); disp(K);
printf("cam_transform (assunto T_rc = robot <- camera) =\n"); disp(T_rc);

% --- load first two measurements
meas0 = load_measurement(files{1});
meas1 = load_measurement(files{2});

% --- mutual matches
th = 0.5;
matches = match_by_appearance_mutual(meas0.app, meas1.app, th);
printf("Mutual matches: %d\n", rows(matches));

% --- robot poses from GT (world <- robot)
T0_wr = v2t_se3_planar(meas0.gt_pose);
T1_wr = v2t_se3_planar(meas1.gt_pose);

% --- camera poses (world <- camera): T_wc = T_wr * T_rc
T0_wc = robotpose_to_camerapose(T0_wr, T_rc);
T1_wc = robotpose_to_camerapose(T1_wr, T_rc);

% --- world -> camera
T0_cw = inv(T0_wc);
T1_cw = inv(T1_wc);

% --- projection matrices
P0 = projection_matrix(K, T0_cw);
P1 = projection_matrix(K, T1_cw);

% --- triangulation
i0 = matches(:,1);
i1 = matches(:,2);

Xw = zeros(3, rows(matches));
keep = true(rows(matches),1);

for k = 1:rows(matches)
  u0 = meas0.uv(i0(k), :)';
  u1 = meas1.uv(i1(k), :)';

  X = linear_triangulation(P0, P1, u0, u1);
  Xw(:,k) = X;

  % cheirality check: profondità positiva
  X0 = T0_cw * [X; 1];
  X1 = T1_cw * [X; 1];

  if X0(3) <= 0 || X1(3) <= 0
    keep(k) = false;
  endif
endfor

Xw = Xw(:, keep);
i0 = i0(keep);

printf("Punti triangolati tenuti (Z>0 in entrambe): %d\n", columns(Xw));

% --- map appearance from frame0
app_map = meas0.app(i0, :)';   % 10xN

state.P = Xw;
state.P_app = app_map;
state.poses = {T0_wc, T1_wc};  % pose CAMERA (world <- camera)

save("-binary", "init_state_gt.mat", "state");
printf("Salvato: init_state_gt.mat\n");

