function measurements = load_measurement_sequence(data_dir, num_frames)
    measurements = cell(num_frames, 1);

    for k = 1:num_frames
        filename = fullfile(data_dir, sprintf('meas-%05d.dat', k-1));
        measurements{k} = load_measurement(filename);
    end
end
