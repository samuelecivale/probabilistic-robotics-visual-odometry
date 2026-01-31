function [match01, best_dist] = match_by_appearance_nn(app0, app1)
  % Per ogni feature in frame0 trova il NN in frame1
  D = appearance_distance(app0, app1);   % N0 x N1
  [best_dist, j] = min(D, [], 2);        % N0x1
  match01 = j;                           % indice in frame1
endfunction

