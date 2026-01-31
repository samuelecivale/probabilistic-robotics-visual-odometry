function J = numeric_jacobian(fun, x0)
  eps = 1e-6;
  f0 = fun(x0);
  m = numel(f0);
  n = numel(x0);
  J = zeros(m, n);

  for i=1:n
    dx = zeros(n,1);
    dx(i) = eps;
    fp = fun(x0 + dx);
    fm = fun(x0 - dx);
    J(:,i) = (fp - fm) / (2*eps);
  endfor
endfunction

