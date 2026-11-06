function test_fgo()
  % Exact recovery and invariance test graph assembly independently of noisy demo.
  truth = [0, 1, 3, 6; 2, 4, 3, 5];
  for delta = [Inf, 1.5]
    [estimate, info] = solve_fgo(truth, diff(truth,1,2), 4, 0.15, delta);
    assert(max(abs(estimate(:)-truth(:))) < 1e-9);
    assert(info.converged);
  end
  data = make_demo_data();
  [estimate, info] = solve_fgo(data.absolute, data.displacement, 4, 0.15, 1.5);
  assert(info.converged);
  assert(all(diff(info.objective) <= 1e-9 * max(1, info.objective(1))));
  shift = [100; -70];
  shifted = solve_fgo(data.absolute + shift, data.displacement, 4, 0.15, 1.5);
  assert(max(abs((shifted-estimate-shift)(:))) < 1e-5);
  rotation = [0, -1; 1, 0];
  rotated = solve_fgo(rotation*data.absolute, rotation*data.displacement, 4, 0.15, 1.5);
  assert(max(abs((rotated-rotation*estimate)(:))) < 1e-5);
  result = demo_fgo(false);
  % Fixture regression only, not a general assertion that robust FGO always wins.
  assert(sum(result.errors(3,:).^2) < sum(result.errors(2,:).^2));
  fprintf('PASS: exact recovery, convergence, monotonic objective, translation, rotation, fixture regression.\n');
end
