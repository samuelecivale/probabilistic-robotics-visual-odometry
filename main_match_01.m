clear; close all; clc;
addpath(genpath("src"));

graphics_toolkit("gnuplot");
setenv("GNUTERM", "pngcairo");

data_dir = "data";
files = list_measurements(data_dir);

meas0 = load_measurement(files{1});
meas1 = load_measurement(files{2});

[match01, dist01] = match_by_appearance_nn(meas0.app, meas1.app);

printf("Frame0 points: %d | Frame1 points: %d\n", rows(meas0.app), rows(meas1.app));
printf("Distanza appearance: min %.4f | median %.4f | mean %.4f | max %.4f\n", ...
       min(dist01), median(dist01), mean(dist01), max(dist01));

% Prova 3 soglie candidate (poi le aggiustiamo)
ths = [0.5, 1.0, 1.5];
for t = ths
  n = sum(dist01 < t);
  printf("  matches con dist < %.2f : %d\n", t, n);
endfor

% (debug) quanto è “giusto” usando true_id (NON per VO finale)
% Nota: questa è solo metrica di qualità dell’association
true_match = meas0.true_id == meas1.true_id(match01);
printf("Accuratezza NN (debug con true_id): %.2f%%\n", 100*mean(true_match));

% Salva istogramma distanze
f = figure("visible","off");
hist(dist01, 30);
xlabel("L2 distance appearance (NN frame0->frame1)");
ylabel("count");
title("Distribuzione distanze appearance (NN)");
print(f, "match01_dist_hist.png", "-dpng", "-r150");
close(f);

printf("Salvato: match01_dist_hist.png\n");

