function camera = load_camera(filename)
    fid = fopen(filename, 'r');
    assert(fid ~= -1, 'Cannot open file: %s', filename);

    camera = struct();
    camera.K = [];
    camera.cam_transform = [];
    camera.z_near = [];
    camera.z_far = [];
    camera.width = [];
    camera.height = [];

    while true
        line = fgetl(fid);
        if ~ischar(line)
            break;
        end

        line = strtrim(line);
        if isempty(line)
            continue;
        end

        if strcmp(line, 'camera matrix:')
            K = zeros(3, 3);
            for i = 1:3
                row = fgetl(fid);
                K(i, :) = sscanf(row, '%f').';
            end
            camera.K = K;

        elseif strcmp(line, 'cam_transform:')
            T = zeros(4, 4);
            for i = 1:4
                row = fgetl(fid);
                T(i, :) = sscanf(row, '%f').';
            end
            camera.cam_transform = T;

        elseif strncmp(line, 'z_near:', 7)
            camera.z_near = sscanf(line(8:end), '%f');

        elseif strncmp(line, 'z_far:', 6)
            camera.z_far = sscanf(line(7:end), '%f');

        elseif strncmp(line, 'width:', 6)
            camera.width = sscanf(line(7:end), '%f');

        elseif strncmp(line, 'height:', 7)
            camera.height = sscanf(line(8:end), '%f');
        end
    end

    fclose(fid);

    assert(~isempty(camera.K) && all(size(camera.K) == [3, 3]), 'Invalid camera matrix');
    assert(~isempty(camera.cam_transform) && all(size(camera.cam_transform) == [4, 4]), 'Invalid cam_transform');
    assert(~isempty(camera.z_near), 'Missing z_near');
    assert(~isempty(camera.z_far), 'Missing z_far');
    assert(~isempty(camera.width), 'Missing width');
    assert(~isempty(camera.height), 'Missing height');
end
