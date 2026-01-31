clear; close all; clc;
warning("off","all");
addpath(genpath("src"));

data_dir = "data";
files = list_measurements(data_dir);

meas0 = load_measurement(files{1});
meas1 = load_measurement(files{2});

% K temporanea (finché non parsiamo camera.dat)
K = [500 0 320;
     0 500 120;
     0   0   1];

% mutual matches (affidabili)
th = 0.5;
matches = match_by_appearance_mutual(meas0.app, meas1.app, th);
printf("Mutual matches: %d\n", rows(matches));

% pose GT (planare -> SE3)
T0_wc = v2t_se3_planar(meas0.gt_pose);   % world <- cam0
T1_wc = v2t_se3_planar(meas1.gt_pose);   % world <- cam1
T0_cw = inv(T0_wc);                      % cam0 <- world
T1_cw = inv(T1_wc);                      % cam1 <- world

P0 = projection_matrix(K, T0_cw);
P1 = projection_matrix(K, T1_cw);

% triangolazione
i0 = matches(:,1);
i1 = matches(:,2);

Xw = zeros(3, rows(matches));
keep = true(rows(matches),1);

for k=1:rows(matches)
  u0 = meas0.uv(i0(k), :)';  % 2x1
  u1 = meas1.uv(i1(k), :)';
  X = linear_triangulation(P0, P1, u0, u1);
  Xw(:,k) = X;

  % Filtra: Z>0 in entrambe le camere (cheirality check)
  X0 = T0_cw * [X;1];
  X1 = T1_cw * [X;1];
  if X0(3) <= 0 || X1(3) <= 0
    keep(k) = false;
  endif
endfor

Xw = Xw(:, keep);
i0 = i0(keep);

printf("Punti triangolati tenuti (Z>0 in entrambe): %d\n", columns(Xw));

% appearance della mappa: prendiamo quella del frame0
app_map = meas0.app(i0, :)';   % 10xN

% Stato iniziale (debug)
state.P = Xw;                 % 3xN
state.P_app = app_map;        % 10xN
state.poses = {T0_wc, T1_wc}; % in debug: GT

save("-binary", "init_state_gt.mat", "state");
printf("Salvato: init_state_gt.mat\n");

