function assoc = associate_frame_to_map_by_appearance(map_desc, frame_desc)
    matches = match_by_appearance_mutual(map_desc, frame_desc);

    assoc = struct();
    assoc.map_idx = matches.idx1;
    assoc.frame_idx = matches.idx2;
    assoc.dist = matches.dist;
end
