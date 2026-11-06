# GSDC 2021 Samsung S20 Ultra full drive

This directory is a local validation copy of the public dynamic GSDC data mirrored
by the AILocAR `androidGnss` project.

- Collection: `2021-04-28-US-SJC-1`
- Phone: `SamsungS20Ultra`
- Duration: 2,083 GNSS seconds (2,081 derived GNSS epochs)
- GNSS file: 53,619 observations plus header
- Ground truth: 2,083 fixes plus header
- Measurement profile: derived pseudorange and satellite state/corrections
- Not present in this mirror: pseudorange rate, C/N0, elevation, raw IMU

Source repository:

- <https://github.com/AILocAR/androidGnss>
- Commit: `1f8d4bf12eedc0ba0efe4dbbcf103cba380a8599`
- GNSS source path:
  `data/GSDC/SJC/2021-04-28-US-SJC-1/SamsungS20Ultra/SamsungS20Ultra_derived.csv`
- Truth source path:
  `data/GSDC/SJC/2021-04-28-US-SJC-1/SamsungS20Ultra/ground_truth.csv`
- Retrieved: 2026-07-17

The source repository is Apache-2.0 and its license is preserved as
`LICENSE.androidGnss`. The mirrored files originate from the Google Smartphone
Decimeter Challenge; use them for local research validation and review the original
competition terms before redistribution.

`device_gnss.csv` is byte-for-byte the source `SamsungS20Ultra_derived.csv`, renamed
only to give replay datasets a consistent filename. Verify with `SHA256SUMS`.

The device timestamp changes from exact integer milliseconds to one millisecond
before the matching truth key partway through the trace. Evaluation intentionally
matches truth within plus or minus 1 ms and does not interpolate positions.
