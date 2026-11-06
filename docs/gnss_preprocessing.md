# GNSS Preprocessing Layer

The GNSS preprocessing layer converts Android raw GNSS replay logs into clean observation epochs for future WLS, EKF, and FGO modules. It does not solve for position and does not own estimator state.

## Implemented Python Path

Entry point:

```bash
python3 python_tools/gnss/preprocess.py tests/data/sample_preprocess_log.ndjson
```

Library API:

- `python_tools.gnss.preprocess.preprocess_log(path, config)`
- `python_tools.gnss.preprocess.preprocess_records(records, config)`
- `python_tools.gnss.preprocess.compute_pseudorange_m(clock, measurement)`
- `python_tools.gnss.coordinates.lla_to_ecef`
- `python_tools.gnss.coordinates.ecef_to_lla`
- `python_tools.gnss.coordinates.ecef_to_enu`
- `python_tools.gnss.coordinates.enu_to_ecef`

## Observation Epoch Output

Each `ObservationEpoch` contains:

- `epoch_elapsed_realtime_nanos`
- source `clock`
- accepted `Observation` records
- rejected measurements with reason strings

Each accepted `Observation` contains:

- constellation type/name and SVID
- pseudorange in meters
- pseudorange uncertainty in meters
- pseudorange-rate/Doppler-derived range-rate in m/s
- pseudorange-rate uncertainty in m/s
- C/N0
- optional carrier frequency
- raw Android state flags for downstream diagnostics

## Filters And Rejection Reasons

The default filter accepts GPS, QZSS, Galileo, and BeiDou on common L1/E1/B1 and L5/E5/B2-style frequency ranges.

Rejection reasons include:

- `missing_satellite_id`
- `constellation_rejected`
- `cn0_below_threshold`
- `frequency_rejected`
- `invalid_time_state`
- `missing_full_bias_nanos`
- `pseudorange_unavailable`
- `pseudorange_out_of_range`
- `pseudorange_uncertainty_too_large`
- `pseudorange_rate_invalid`
- `pseudorange_rate_uncertainty_too_large`

## Pseudorange Policy

Pseudorange is computed from Android raw GNSS clock fields only when `full_bias_nanos`, `time_nanos`, valid satellite receive time, and constellation time-lock state are available.

Current implementation prepares code pseudorange observations for replay and baseline estimators. It does not apply satellite ephemeris, satellite clock correction, ionosphere, troposphere, Earth rotation, relativistic correction, or inter-signal bias corrections. Those are exposed through the C++ ephemeris/correction interface and should be applied by future GNSS modeling code.

## C++ Interfaces

Future C++ consumers should use:

- `s23::GnssObservation`
- `s23::GnssObservationEpoch`
- `s23::RejectedGnssMeasurement`
- `s23::IEphemerisCorrectionProvider`
- `s23::SatelliteCorrection`
- `s23::Lla`, `s23::Ecef`, `s23::Enu`

The ephemeris/correction provider interface deliberately returns corrected satellite position, velocity, satellite clock, and atmospheric/model corrections without specifying the source. A later task can back it with broadcast ephemeris, precise products, SSR/RTK corrections, or test fixtures.
