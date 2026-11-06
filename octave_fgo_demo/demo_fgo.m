function result = demo_fgo(show_plots)
  % Run demo_fgo() interactively or demo_fgo(false) without graphics.
  if nargin < 1, show_plots = true; end
  data = make_demo_data();
  [least_squares, ls_info] = solve_fgo(data.absolute, data.displacement, ...
    data.sigma_absolute, data.sigma_relative, Inf);
  [robust, robust_info] = solve_fgo(data.absolute, data.displacement, ...
    data.sigma_absolute, data.sigma_relative, data.huber_delta);
  estimates = {data.absolute, least_squares, robust};
  labels = {'Absolute measurements', 'Gaussian FGO', 'Huber FGO'};
  errors = zeros(3, columns(data.truth));
  fprintf('SYNTHETIC 2D example: these are not smartphone accuracy results.\n');
  for k = 1:3
    errors(k,:) = sqrt(sum((estimates{k}-data.truth).^2, 1));
    fprintf('%-23s horizontal RMSE: %.3f m\n', labels{k}, sqrt(mean(errors(k,:).^2)));
  end
  fprintf('Huber iterations: %d; converged: %d\n', robust_info.iterations, robust_info.converged);
  if show_plots
    figure('name', 'Synthetic factor graph optimization');
    subplot(2,1,1);
    plot(data.truth(1,:), data.truth(2,:), 'k-', ...
         data.absolute(1,:), data.absolute(2,:), 'r.', ...
         least_squares(1,:), least_squares(2,:), 'b--', ...
         robust(1,:), robust(2,:), 'g-', 'linewidth', 1.3);
    axis equal; grid on;
    xlabel('East [m]'); ylabel('North [m]');
    title('Synthetic trajectory: absolute + relative displacement factors');
    legend([{'Synthetic truth'}, labels], 'location', 'best');
    subplot(2,1,2);
    plot(data.time, errors'); grid on;
    xlabel('Time [s]'); ylabel('Horizontal error [m]'); legend(labels);
  end
  result = struct('data', data, 'least_squares', least_squares, ...
    'robust', robust, 'errors', errors, 'ls_info', ls_info, 'robust_info', robust_info);
end
