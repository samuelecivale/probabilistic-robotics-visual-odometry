clear; close all; clc;
warning("off","all");              % <— leva warning
addpath(genpath("src"));

data_dir = "data";
files = list_measurements(data_dir);

meas0 = load_measurement(files{1});
meas1 = load_measurement(files{2});

th = 0.5;
matches = match_by_appearance_mutual(meas0.app, meas1.app, th);

printf("Mutual matches (th=%.2f): %d\n", th, rows(matches));

% Debug accuracy con true_id (non per VO)
i0 = matches(:,1);
i1 = matches(:,2);
acc = mean(meas0.true_id(i0) == meas1.true_id(i1));
printf("Accuratezza mutual (debug con true_id): %.2f%%\n", 100*acc);

