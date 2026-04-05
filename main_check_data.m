clear;
clc;
close all;

data_dir = 'data';

fprintf('=== DATASET CHECK ===\n');

camera_file = fullfile(data_dir, 'camera.dat');
traj_file   = fullfile(data_dir, 'trajectory.dat');
world_file  = fullfile(data_dir, 'world.dat');

assert(exist(camera_file, 'file') == 2, 'camera.dat not found');
assert(exist(traj_file, 'file') == 2, 'trajectory.dat not found');
assert(exist(world_file, 'file') == 2, 'world.dat not found');

fprintf('Found camera.dat, trajectory.dat, world.dat\n');

meas_files = dir(fullfile(data_dir, 'meas-*.dat'));
num_meas = numel(meas_files);
fprintf('Found %d measurement files\n', num_meas);

assert(num_meas > 0, 'No measurement files found');

preview_list = {
    camera_file, 15, 'camera.dat';
    traj_file,   10, 'trajectory.dat';
    world_file,  10, 'world.dat';
    fullfile(data_dir, meas_files(1).name), 15, meas_files(1).name
};

if num_meas >= 2
    preview_list(end+1, :) = {fullfile(data_dir, meas_files(2).name), 15, meas_files(2).name};
end

for k = 1:size(preview_list, 1)
    filename = preview_list{k, 1};
    nlines   = preview_list{k, 2};
    label    = preview_list{k, 3};

    fprintf('\n--- RAW PREVIEW: %s ---\n', label);

    fid = fopen(filename, 'r');
    assert(fid ~= -1, 'Cannot open file: %s', filename);

    for i = 1:nlines
        line = fgetl(fid);
        if ~ischar(line)
            break;
        end
        fprintf('%s\n', line);
    end

    fclose(fid);
end

fprintf('\n=== CHECK COMPLETED ===\n');
