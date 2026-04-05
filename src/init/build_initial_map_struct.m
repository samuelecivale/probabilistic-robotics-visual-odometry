function [map, init] = build_initial_map_struct(camera, meas0, meas1)
    init = build_initial_map_from_two_view(camera, meas0, meas1);

    n = size(init.Pw_map, 1);

    map = struct();
    map.points = init.Pw_map;
    map.desc = init.map_desc;
    map.obs_count = 2 * ones(n, 1);
    map.created_frame = 2 * ones(n, 1);
    map.last_seen = 2 * ones(n, 1);
end
