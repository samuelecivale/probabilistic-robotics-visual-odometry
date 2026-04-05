function init = build_initial_map_from_two_view(camera, meas0, meas1)
    matches = match_by_appearance_mutual(meas0.appearance, meas1.appearance);

    uv0 = meas0.uv(matches.idx1, :);
    uv1 = meas1.uv(matches.idx2, :);

    x0 = pixels_to_normalized(camera.K, uv0);
    x1 = pixels_to_normalized(camera.K, uv1);

    E = estimate_essential_eight_point(x0, x1);
    candidates = decompose_essential(E);
    result = select_essential_pose(candidates, x0, x1);

    valid = result.valid_cheirality;

    init = struct();
    init.E = E;
    init.matches = matches;
    init.R01 = result.R;
    init.t01 = result.t / norm(result.t);
    init.Pw_map = result.Pw(valid, :);
    init.map_desc = meas0.appearance(matches.idx1(valid), :);
    init.map_ids = meas0.point_id(matches.idx1(valid));
    init.num_map_points = size(init.Pw_map, 1);
end
