function [local_map, map_idx] = select_local_map(map, current_frame, params)
    if nargin < 3
        params = struct();
    end

    if ~isfield(params, 'recent_window'); params.recent_window = 20; end
    if ~isfield(params, 'min_obs_keep');  params.min_obs_keep = 4;   end
    if ~isfield(params, 'max_points');    params.max_points = 300;   end

    n = size(map.points, 1);

    is_recent = (current_frame - map.last_seen) <= params.recent_window;
    is_reliable = map.obs_count >= params.min_obs_keep;

    keep = is_recent | is_reliable;

    idx = find(keep);

    if isempty(idx)
        idx = (1:n).';
    end

    % score: prefer recent + frequently observed
    age = current_frame - map.last_seen(idx);
    score = map.obs_count(idx) - 0.1 * age;

    [~, order] = sort(score, 'descend');
    idx = idx(order);

    if numel(idx) > params.max_points
        idx = idx(1:params.max_points);
    end

    map_idx = idx;

    local_map = struct();
    local_map.points = map.points(idx, :);
    local_map.desc = map.desc(idx, :);
    local_map.obs_count = map.obs_count(idx, :);
    local_map.created_frame = map.created_frame(idx, :);
    local_map.last_seen = map.last_seen(idx, :);
end
