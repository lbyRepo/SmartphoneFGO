# GSDC validation

## Full Samsung drive

Run the sustained public benchmark:

```bash
python3 validation_tools/gsdc_benchmark.py \
  --device-gnss datasets/gsdc_2021_s20ultra_sjc/device_gnss.csv \
  --ground-truth datasets/gsdc_2021_s20ultra_sjc/ground_truth.csv \
  --output validation_results/gsdc_2021_s20ultra_sjc \
  --dataset-name "GSDC 2021 / Samsung S20 Ultra / SJC full drive" \
  --dataset-scope "2,083-second public dynamic drive; derived pseudorange only"
```

This trace provides derived pseudorange and satellite states but no Doppler or IMU.
It is used to measure sustained GNSS-only WLS/FGO accuracy, smoothness, convergence,
and latency. See `fgo_android_app/docs/validation_report.md` for results and source
provenance.

## Rich-schema smoke fixture

Run the regression benchmark with the bundled dependency runtime:

```bash
/Users/leeboonyao/.cache/codex-runtimes/codex-primary-runtime/dependencies/python/bin/python3 \
  validation_tools/gsdc_benchmark.py \
  --device-gnss datasets/gsdc_2023_sample/device_gnss.csv \
  --ground-truth datasets/gsdc_2023_sample/ground_truth.csv \
  --output validation_results/gsdc_2023_sample
```

The command writes `solutions.csv`, `summary.json`, and `trajectory.svg`.

This fixture is only five seconds long. It verifies parsing, corrections, coordinate
frames, numerical Jacobians, solver convergence, output stability, and reproducible
metrics. It does not validate Samsung S23+ hardware behavior or establish outdoor
accuracy. Those claims require full traces plus real S23+ field logs.

Validate an exported Android log with:

```bash
python3 validation_tools/validate_ndjson.py device_logs/<session>.ndjson
```

Replay the exported log through causal GPS L1 C/A preprocessing and, when at
least four clean observations are available, the shared WLS/FGO solvers:

```bash
python3 validation_tools/navgraph_replay.py device_logs/<session>.ndjson \
  --output validation_results/<session>
```

The command writes `clean_observations.csv`, `imu.csv`, `fused_location.csv`,
`solutions.csv`, `summary.json`, and `trajectory.svg`. Android fused location is
reported as a comparison baseline, never as ground truth. A log without complete
broadcast ephemerides or physically valid pseudoranges still produces diagnostic
artifacts and an explicit `not_run_no_clean_epochs` solver status.
