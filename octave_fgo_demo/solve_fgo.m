function [position, info] = solve_fgo(absolute, displacement, sigma_absolute, sigma_relative, huber_delta)
  % Batch 2D factor graph. Columns are epochs; rows are east/north [m].
  % huber_delta = Inf gives ordinary weighted least squares.
  if nargin < 5, huber_delta = 1.5; end
  n = columns(absolute);
  assert(n >= 2 && rows(absolute) == 2 && all(isfinite(absolute(:))));
  assert(isequal(size(displacement), [2, n-1]) && all(isfinite(displacement(:))));
  assert(isscalar(sigma_absolute) && isfinite(sigma_absolute) && sigma_absolute > 0);
  assert(isscalar(sigma_relative) && isfinite(sigma_relative) && sigma_relative > 0);
  assert(isscalar(huber_delta) && huber_delta > 0);

  % Each row block is a whitened factor: A*x - b.
  difference = sparse(1:n-1, 1:n-1, -1, n-1, n) ...
             + sparse(1:n-1, 2:n, 1, n-1, n);
  A = [speye(2*n) / sigma_absolute; ...
       kron(difference, speye(2)) / sigma_relative];
  b = [absolute(:) / sigma_absolute; displacement(:) / sigma_relative];
  x = absolute(:);
  history = objective(A*x-b, n, huber_delta);
  converged = false;
  for iteration = 1:100
    residual = reshape((x - absolute(:)) / sigma_absolute, 2, n);
    norms = sqrt(sum(residual.^2, 1));
    weights = ones(1, n);
    if isfinite(huber_delta)
      weights = min(1, huber_delta ./ max(norms, eps));
    end
    % One Huber weight per 2D absolute factor (rotation invariant).
    scale = [kron(sqrt(weights(:)), ones(2, 1)); ones(2*(n-1), 1)];
    W = spdiags(scale, 0, length(scale), length(scale));
    candidate = (W*A) \ (W*b); % Sparse least squares, no explicit inverse.
    history(end+1) = objective(A*candidate-b, n, huber_delta);
    step = norm(candidate-x, Inf);
    x = candidate;
    if step < 1e-8 * (1 + norm(x, Inf))
      converged = true;
      break;
    end
  end
  position = reshape(x, 2, n);
  info = struct('iterations', iteration, 'converged', converged, ...
                'objective', history);
end

function cost = objective(residual, n, delta)
  absolute_norm = sqrt(sum(reshape(residual(1:2*n), 2, n).^2, 1));
  loss = 0.5 * absolute_norm.^2;
  if isfinite(delta)
    outside = absolute_norm > delta;
    loss(outside) = delta * (absolute_norm(outside) - 0.5*delta);
  end
  cost = sum(loss) + 0.5*sum(residual(2*n+1:end).^2);
end
