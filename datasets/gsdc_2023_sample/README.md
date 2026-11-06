# GSDC 2023 sample fixture

This directory contains a small public trace from the Stanford NavLab `gnss_lib_py`
test data mirror of the Google Smartphone Decimeter Challenge 2023 dataset. It is
intended for parser and deterministic regression tests, not for publishing broad
accuracy claims.

Trace: `2023-09-07-18-59-us-ca/pixel7pro`

Source files:

- `device_gnss.csv`: derived smartphone GNSS observations.
- `ground_truth.csv`: corresponding receiver ground truth.

Upstream source:
`https://github.com/Stanford-NavLab/gnss_lib_py/tree/main/data/unit_test/google_decimeter_2023/2023-09-07-18-59-us-ca/pixel7pro`

The complete challenge dataset remains available through Kaggle under its own
competition terms. Record checksums in `SHA256SUMS` after downloading so benchmark
runs are tied to exact inputs.
