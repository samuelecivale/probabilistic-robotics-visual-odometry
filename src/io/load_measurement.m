function meas = load_measurement(filename)
    fid = fopen(filename, 'r');
    assert(fid ~= -1, 'Cannot open file: %s', filename);

    seq = [];
    gt_pose = [];
    odom_pose = [];
    point_rows = [];

    while true
        line = fgetl(fid);
        if ~ischar(line)
            break;
        end

        line = strtrim(line);
        if isempty(line)
            continue;
        end

        if strncmp(line, 'seq:', 4)
            seq = sscanf(line(5:end), '%f');

        elseif strncmp(line, 'gt_pose:', 8)
            gt_pose = sscanf(line(9:end), '%f').';

        elseif strncmp(line, 'odom_pose:', 10)
            odom_pose = sscanf(line(11:end), '%f').';

        elseif strncmp(line, 'point ', 6)
            nums = sscanf(line(7:end), '%f').';
            point_rows = [point_rows; nums];
        end
    end

    fclose(fid);

    meas = struct();
    meas.seq = seq;
    meas.gt_pose = gt_pose;
    meas.odom_pose = odom_pose;

    if isempty(point_rows)
        meas.point_local_id = zeros(0, 1);
        meas.point_id = zeros(0, 1);
        meas.uv = zeros(0, 2);
        meas.appearance = zeros(0, 10);
        return;
    end

    assert(size(point_rows, 2) >= 14, 'Each point row must have at least 14 numeric values');

    meas.point_local_id = point_rows(:, 1);
    meas.point_id = point_rows(:, 2);
    meas.uv = point_rows(:, 3:4);
    meas.appearance = point_rows(:, 5:end);
end
