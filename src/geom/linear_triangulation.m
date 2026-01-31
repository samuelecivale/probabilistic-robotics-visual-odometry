function X = linear_triangulation(P0, P1, u0, u1)
  x0 = u0(1); y0 = u0(2);
  x1 = u1(1); y1 = u1(2);

  A = [
    x0*P0(3,:) - P0(1,:);
    y0*P0(3,:) - P0(2,:);
    x1*P1(3,:) - P1(1,:);
    y1*P1(3,:) - P1(2,:);
  ];

  [~, ~, V] = svd(A);
  Xh = V(:, end);
  X = Xh(1:3) / Xh(4);
endfunction

