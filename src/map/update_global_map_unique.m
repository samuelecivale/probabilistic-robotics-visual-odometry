function [map, stats] = update_global_map_unique(map, points_new, desc_new, current_frame, max_desc_dist)
    if nargin < 5
        max_desc_dist = 1e-8;
    end

    if ~isfield(map, 'points');         map.points = zeros(0,3); end
    if ~isfield(map, 'desc');           map.desc = zeros(0,10); end
    if ~isfield(map, 'obs_count');      map.obs_count = zeros(0,1); end
    if ~isfield(map, 'created_frame');  map.created_frame = zeros(0,1); end
    if ~isfield(map, 'last_seen');      map.last_seen = zeros(0,1); end

    added = 0;
    updated = 0;

    for i = 1:size(points_new, 1)
        p = points_new(i, :);
        d = desc_new(i, :);

        found = false;
        best_idx = 0;

        if ~isempty(map.desc)
            D2 = sum((map.desc - d).^2, 2);
            [best_d2, best_idx] = min(D2);
            if sqrt(best_d2) <= max_desc_dist
                found = true;
            end
        end

        if found
            nold = map.obs_count(best_idx);
            map.points(best_idx, :) = (nold * map.points(best_idx, :) + p) / (nold + 1);
            map.obs_count(best_idx) = nold + 1;
            map.last_seen(best_idx) = current_frame;
            updated = updated + 1;
        else
            map.points(end+1, :) = p;
            map.desc(end+1, :) = d;
            map.obs_count(end+1, 1) = 1;
            map.created_frame(end+1, 1) = current_frame;
            map.last_seen(end+1, 1) = current_frame;
            added = added + 1;
        end
    end

    stats = struct();
    stats.added = added;
    stats.updated = updated;
end
