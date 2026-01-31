function matches = match_by_appearance_mutual(app0, app1, th)
  % matches: Kx2, ogni riga = [i0, i1]
  % Mutual nearest neighbor + threshold sulla distanza

  % 0 -> 1
  D01 = appearance_distance(app0, app1);
  [d01, j01] = min(D01, [], 2);

  % 1 -> 0
  D10 = appearance_distance(app1, app0);
  [d10, j10] = min(D10, [], 2);

  pairs = [];
  for i0 = 1:rows(app0)
    i1 = j01(i0);
    if d01(i0) < th && j10(i1) == i0
      pairs(end+1, :) = [i0, i1]; %#ok<AGROW>
    endif
  endfor

  matches = pairs;
endfunction

