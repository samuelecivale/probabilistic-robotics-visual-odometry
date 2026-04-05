function assoc = extract_active_correspondences(image_matches, active_prev_img_idx)
    assoc = struct();
    assoc.active_idx = zeros(0,1);
    assoc.prev_idx   = zeros(0,1);
    assoc.curr_idx   = zeros(0,1);

    if isempty(active_prev_img_idx) || isempty(image_matches.idx1)
        return;
    end

    max_idx = max([image_matches.idx1(:); active_prev_img_idx(:)]);
    lut = zeros(max_idx, 1);

    for i = 1:numel(active_prev_img_idx)
        idx = active_prev_img_idx(i);
        if idx >= 1 && idx <= max_idx
            lut(idx) = i;
        end
    end

    keep_active = [];
    keep_prev = [];
    keep_curr = [];

    for k = 1:numel(image_matches.idx1)
        prev_idx = image_matches.idx1(k);
        curr_idx = image_matches.idx2(k);

        if prev_idx < 1 || prev_idx > numel(lut)
            continue;
        end

        active_idx = lut(prev_idx);
        if active_idx > 0
            keep_active(end+1, 1) = active_idx;
            keep_prev(end+1, 1) = prev_idx;
            keep_curr(end+1, 1) = curr_idx;
        end
    end

    assoc.active_idx = keep_active;
    assoc.prev_idx = keep_prev;
    assoc.curr_idx = keep_curr;
end
