# Replay And Evaluation

The replay layer provides one command that turns a dataset into consistent estimator outputs, metrics, and plots.

## Synthetic Dataset

Synthetic replay runs all current estimators on the same deterministic data:

```bash
python3 python_tools/replay/run_replay.py \
  --dataset synthetic \
  --output-dir replay_outputs/synthetic
```

Generated artifacts:

- `truth.csv`
- `android_fused.csv`
- `estimates_wls.csv`
- `estimates_ekf.csv`
- `estimates_fgo.csv`
- `solver_iterations.csv`
- `metrics_summary.json`
- `trajectory_xy.svg`
- `position_error.svg`

The current synthetic path compares:

- Android fused baseline
- WLS pseudorange baseline
- Minimal EKF baseline
- GNSS-only fixed-lag FGO with pseudorange and Doppler/range-rate factors

Metrics include position/horizontal error when truth exists, trajectory smoothness, solver latency, update rate, solver iterations, solver failures, and position jumps.

## Samsung S23+ Logs

Raw Android logs can be replayed through the common parser and GNSS preprocessing layer:

```bash
python3 python_tools/replay/run_replay.py \
  --dataset log \
  --input tests/data/sample_preprocess_log.ndjson \
  --output-dir replay_outputs/log_sample
```

Generated artifacts:

- `observations.csv`
- `rejected_measurements.csv`
- `android_fused.csv`
- `metrics_summary.json`

Positioning estimators are not run on real phone logs yet because satellite ephemeris/correction support is only an interface at this stage. The log replay path still verifies deterministic parsing, preprocessing, rejection reasons, and fused-location extraction.

## Reproducibility Contract

For a fixed input file and code revision, replay outputs should be byte-stable except for floating-point formatting changes caused by Python runtime differences. Estimator output CSVs use one row per epoch and one file per estimator to make WLS/EKF/FGO comparisons scriptable.
