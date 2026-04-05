function map = prune_map_light(map, current_frame, params)
    if nargin < 3
        params = struct();
    end

    if ~isfield(params, 'max_age');      params.max_age = 40;  end
    if ~isfield(params, 'min_obs');      params.min_obs = 3;   end
    if ~isfield(params, 'target_size');  params.target_size = 600; end

    n = size(map.points, 1);
    if n <= params.target_size
        return;
    end

    age = current_frame - map.last_seen;
    weak = (map.obs_count < params.min_obs) & (age > params.max_age);

    keep = ~weak;

    if sum(keep) < params.target_size
        % fallback: keep best scoring landmarks
        score = map.obs_count - 0.1 * age;
        [~, order] = sort(score, 'descend');
        keep = false(n,1);
        keep(order(1:min(params.target_size, n))) = true;
    end

    map.points = map.points(keep, :);
    map.desc = map.desc(keep, :);
    map.obs_count = map.obs_count(keep, :);
    map.created_frame = map.created_frame(keep, :);
    map.last_seen = map.last_seen(keep, :);
end
