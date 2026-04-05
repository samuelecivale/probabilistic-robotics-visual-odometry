clear;
clc;
close all;

addpath(genpath('src'));

meas0 = load_measurement(fullfile('data', 'meas-00000.dat'));
meas1 = load_measurement(fullfile('data', 'meas-00001.dat'));

fprintf('=== APPEARANCE MATCHING TEST ===\n');
fprintf('Frame 0 points: %d\n', numel(meas0.point_id));
fprintf('Frame 1 points: %d\n', numel(meas1.point_id));

nn_matches = match_by_appearance_nn(meas0.appearance, meas1.appearance);
mutual_matches = match_by_appearance_mutual(meas0.appearance, meas1.appearance);

fprintf('\n[Nearest Neighbor]\n');
fprintf('num matches: %d\n', numel(nn_matches.idx1));

nn_correct = (meas0.point_id(nn_matches.idx1) == meas1.point_id(nn_matches.idx2));
fprintf('correct matches: %d / %d\n', sum(nn_correct), numel(nn_correct));
fprintf('accuracy: %.4f\n', mean(nn_correct));
fprintf('mean descriptor distance: %.6f\n', mean(nn_matches.dist));
fprintf('median descriptor distance: %.6f\n', median(nn_matches.dist));

fprintf('\n[Mutual Nearest Neighbor]\n');
fprintf('num matches: %d\n', numel(mutual_matches.idx1));

mutual_correct = (meas0.point_id(mutual_matches.idx1) == meas1.point_id(mutual_matches.idx2));
fprintf('correct matches: %d / %d\n', sum(mutual_correct), numel(mutual_correct));
fprintf('accuracy: %.4f\n', mean(mutual_correct));
fprintf('mean descriptor distance: %.6f\n', mean(mutual_matches.dist));
fprintf('median descriptor distance: %.6f\n', median(mutual_matches.dist));

common_ids = intersect(meas0.point_id, meas1.point_id);
fprintf('\n[GT overlap]\n');
fprintf('common landmark ids between frames: %d\n', numel(common_ids));

if exist('results', 'dir') ~= 7
    mkdir('results');
end

try
    graphics_toolkit('gnuplot');
catch
end

try
    f = figure('visible', 'off');

    subplot(2,1,1);
    hist(nn_matches.dist, 30);
    title('NN descriptor distances');
    xlabel('distance');
    ylabel('count');
    grid on;

    subplot(2,1,2);
    hist(mutual_matches.dist, 30);
    title('Mutual NN descriptor distances');
    xlabel('distance');
    ylabel('count');
    grid on;

    saveas(f, fullfile('results', 'appearance_matching_hist.png'));
    close(f);
    fprintf('\nSaved plot to results/appearance_matching_hist.png\n');
catch err
    fprintf('\nPlot saving skipped: %s\n', err.message);
end

num_examples = min(10, numel(mutual_matches.idx1));
fprintf('\n[First %d mutual matches]\n', num_examples);
for k = 1:num_examples
    i = mutual_matches.idx1(k);
    j = mutual_matches.idx2(k);
    fprintf('  meas0 row %d (id=%d) <-> meas1 row %d (id=%d), dist=%.6f\n', ...
        i, meas0.point_id(i), j, meas1.point_id(j), mutual_matches.dist(k));
end

fprintf('\n=== MATCHING TEST COMPLETED ===\n');
