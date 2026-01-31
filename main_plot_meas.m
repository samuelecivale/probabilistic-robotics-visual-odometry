clear; close all; clc;
addpath(genpath("src"));

% Forza toolkit non-Qt
graphics_toolkit("gnuplot");
setenv("GNUTERM", "pngcairo");   % output stabile

data_dir = "data";
files = list_measurements(data_dir);

meas = load_measurement(files{1});
uv = meas.uv;

f = figure("visible", "off"); hold on; grid on;
plot(uv(:,1), uv(:,2), '.');
xlabel("col (u)");
ylabel("row (v)");
title(sprintf("Feature 2D - %s", files{1}));
set(gca, "YDir", "reverse");
axis equal;

out = "frame0_features.png";
print(f, out, "-dpng", "-r150");
close(f);

printf("Salvato: %s\n", out);

