clear; close all; clc;
addpath(genpath("src"));

data_dir = "data";
files = list_measurements(data_dir);

printf("Trovati %d file di misura.\n", numel(files));

n_show = min(3, numel(files));
for i = 1:n_show
  meas = load_measurement(files{i});
  uv = meas.uv;

  printf("\n[%s]\n", files{i});
  printf("  seq: %d\n", meas.seq);
  printf("  #points: %d\n", rows(uv));
  printf("  uv col: min %.2f  max %.2f\n", min(uv(:,1)), max(uv(:,1)));
  printf("  uv row: min %.2f  max %.2f\n", min(uv(:,2)), max(uv(:,2)));
  printf("  app dim: %d\n", columns(meas.app));
  printf("  gt_pose:  [%.3f %.3f %.3f]\n", meas.gt_pose(1), meas.gt_pose(2), meas.gt_pose(3));
  printf("  odom_pose:[%.3f %.3f %.3f]\n", meas.odom_pose(1), meas.odom_pose(2), meas.odom_pose(3));
endfor

