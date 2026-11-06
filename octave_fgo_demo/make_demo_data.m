function data = make_demo_data()
  % Original synthetic teaching fixture; no competition data or ground-truth tuning.
  previous_rng = rng();
  unwind_protect
    rng(42, 'twister');
    data.time = 0:1:119; % seconds, uniformly spaced synthetic epochs
    t = data.time;
    data.truth = [1.2*t; 12*sin(t/18) + 0.15*t]; % local east/north [m]
    data.sigma_absolute = 4.0; % chosen simulation noise, per axis [m]
    data.sigma_relative = 0.15;
    data.absolute = data.truth + data.sigma_absolute*randn(size(data.truth));
    data.outliers = [25, 26, 60, 90];
    data.absolute(:, data.outliers) += [28; -22]*ones(1, length(data.outliers));
    data.displacement = diff(data.truth, 1, 2) ...
                      + data.sigma_relative*randn(2, columns(data.truth)-1);
    data.huber_delta = 1.5; % fixed threshold in whitened units
  unwind_protect_cleanup
    rng(previous_rng);
  end_unwind_protect
end
