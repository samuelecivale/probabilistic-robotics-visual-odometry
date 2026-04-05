function map = update_map_observations(map, map_idx, current_frame)
    if isempty(map_idx)
        return;
    end

    map.obs_count(map_idx) = map.obs_count(map_idx) + 1;
    map.last_seen(map_idx) = current_frame;
end
