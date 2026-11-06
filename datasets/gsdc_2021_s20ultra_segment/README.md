# GSDC 2021 Samsung S20 Ultra Android replay segment

This is a deterministic 600-second dynamic segment derived from the full local
validation copy in `../gsdc_2021_s20ultra_sjc`.

- Inclusive time selection: `1303675974000` through `1303676573000` GPS ms
- 600 GNSS epochs
- 15,554 derived GNSS observations
- 600 matching ground-truth fixes
- Selection command: retain the CSV header and rows whose third column is inside
  the inclusive time range above

The segment contains moving and stopped vehicle periods and crosses the source's
1 ms GNSS/truth key offset. It is bundled in the Android APK to test sustained
on-device replay while keeping asset size bounded. It has derived pseudorange but
no pseudorange-rate or IMU stream, so it validates GNSS-only WLS/FGO behavior.

See `../gsdc_2021_s20ultra_sjc/README.md` for source, provenance, and licensing.
