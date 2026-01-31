function d = appearance_distance(A, B)
  % A: Nx10, B: Mx10
  % ritorna matrice NxM delle distanze L2
  % (ok per N~127, M~124)
  AA = sum(A.^2, 2);            % Nx1
  BB = sum(B.^2, 2)';           % 1xM
  d2 = AA + BB - 2*(A*B');      % NxM
  d2(d2 < 0) = 0;               % numerica
  d = sqrt(d2);
endfunction

