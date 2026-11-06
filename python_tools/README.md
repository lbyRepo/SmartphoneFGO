# Python Tools

Python tools operate on exported logs and offline datasets. They must not depend on Android runtime APIs.

Current tools:

- `replay/parse_log.py`: strict NDJSON schema validator and record counter.
- `replay/run_replay.py`: single-command synthetic/log replay with estimator CSVs, metrics, and plots.
- `gnss/preprocess.py`: converts raw GNSS replay logs into clean observation epochs with rejection reasons.
- `gnss/coordinates.py`: WGS-84 LLA/ECEF/ENU coordinate transforms.
