# Emulator verification

Verified on the configured `Medium_Phone_API_36.1` Android 16 emulator at
1080 by 2400 pixels using emulator 36.2.12. An earlier host/emulator revision
required `-gpu host -feature -Vulkan` for inspectable screenshots. The 0.9.0 cold
run rendered correctly with the default host graphics selection.

Checks completed:

- Fresh install and runtime permission dialog.
- Live, replay, and diagnostics tabs render without overlap after transitions settle.
- Foreground service starts and stops from the UI.
- Replay parses a bundled 600-second public Samsung S20 Ultra segment and executes
  600 WLS epochs plus two-second fixed-lag FGO on a background thread in the APK;
  displayed metrics are computed, not hard-coded.
- The same replay run executes a deterministic 20-second GNSS/INS scenario with
  2,001 IMU samples, 21 GNSS epochs, EKF propagation, and 20 inertial graph factors.
- Replay retry and the sequence Replay -> Live -> Diagnostics -> Replay complete
  without an app exception or stale service.
- Injected Singapore position appears in the baseline solution and trajectory.
- Closed log contains session boundaries plus IMU, raw GNSS, GNSS clock, and baseline
  location records.
- Strict NDJSON timestamp validation passes with no errors.
- Twenty-eight JVM tests cover WGS84 transforms, ENU axes, quaternion rotation, stationary
  gravity cancellation, gyro propagation, rich and legacy GSDC schemas, tolerant
  truth-time matching, inertial smoothing, innovation rejection, and numerical
  pseudorange/Doppler Jacobians. New live tests cover bounded queue draining,
  Android clock/TOW pseudorange construction, stable preprocessing rejections, and
  deterministic/stale satellite snapshot handling.
- `:app:testDebugUnitTest`, `:app:assembleDebug`, and `:app:lintDebug` pass.

The final 0.5.0 smoke session contained 12,642 aggregate IMU events, 63 GNSS clock
records, 441 raw GNSS measurements, and 110 provider location records. It contained
exactly one session start and end and passed strict NDJSON timestamp validation with
no errors. Emulator GNSS values are synthetic and are not evidence of Samsung S23+
radio performance.

The 0.6.0 live-path verification installed version code 5 and exercised the new
typed collector queue for 74 seconds. Diagnostics observed 4,678 processed events,
zero queued events, and zero drops during the screenshot. Emulator raw GNSS was
rejected as `invalid pseudorange`, which is expected for its artificial receiver
clock; zero clean epochs were reported and the production ephemeris source was
explicitly labeled not configured. The app did not claim a live position or solver.

The closed 0.6.0 NDJSON session contained 15,592 records: 14,868 IMU events, 74 GNSS
clock records, 518 raw measurements, 128 provider locations, two sensor metadata
records, and exactly one start/end pair. Strict validation returned `valid: true`
with no errors. Logcat contained no NavGraph fatal exception or ANR.

A final post-audit smoke log verified the additive `time_offset_nanos` raw GNSS
field. Its 5,791 records included 5,529 IMU events, 27 GNSS clocks, 189 raw
measurements, 42 provider locations, both metadata records, and clean session
boundaries; strict validation again passed without errors.

The 0.7.0 broadcast-ephemeris verification installed version code 6 on a cold,
snapshot-disabled Android 16 boot. The Diagnostics layout added navigation-stream
and GPS L1 C/A provider rows without overlap. At capture time it showed 10,037 IMU
events, 50 raw GNSS epochs, 350 measurements, 10,175 processed typed events, zero
queued, zero dropped, zero navigation messages, and zero complete GPS ephemerides.
The API 36 emulator does not synthesize `GnssNavigationMessage` callbacks, so this
run validates lifecycle, zero-capability handling, and UI truthfulness rather than
broadcast decoding. The three-subframe decoder/orbit path is covered by fixed JVM
regressions instead.

The closed 0.7.0 session contained 14,470 records: 13,798 IMU events, 68 GNSS
clocks, 476 raw measurements, 124 provider locations, two metadata records, and
one start/end pair. The extended validator returned `valid: true` with no errors;
logcat contained no NavGraph fatal exception or ANR. An earlier saved-snapshot boot
became system-shell unresponsive; a cold `-no-snapshot` repeat completed normally,
with no evidence tying the guest failure to the app.

The 0.8.0 live-estimator verification installed version code 7 on another cold,
snapshot-disabled Android 16 boot. The Live screen replaced the unavailable-solver
placeholder with estimator-ready status, while Diagnostics added bounded IMU-pair,
warm-start WLS/EKF, and asynchronous fixed-lag FGO health. At capture time it
showed 799 paired samples, zero unmatched samples, 3.59 ms maximum accelerometer/
gyro skew, 1,619 typed events processed, and no queue drops. All new rows wrapped
within their panels at 1080 by 2400 without clipping or overlap.

The emulator again emitted no navigation messages and nonphysical pseudorange
timing, so the backend remained `waiting for clean epoch` and the Live screen
correctly showed Android fused location rather than claiming a WLS/EKF fix. The
closed schema-v1 session contained 13,140 IMU records, 65 GNSS clocks, 455 raw
measurements, 122 fused fixes, two metadata records, and one start/end pair. Strict
validation returned `valid: true`; deterministic raw replay found one clock arc,
100 Hz per IMU sensor, zero clean epochs, and
`solver_status=not_run_no_clean_epochs`. The service stopped cleanly and filtered
logcat contained no fatal exception or ANR.

The first 0.9.0 feedback verification installed an interim Kotlin-only version code
9 APK with SHA-256
`ee367d82ec26c06eec89d41175cf42a5b26b8319858c9c12640c048268ca351b` on a cold
API 36.1 boot. Live and Diagnostics rendered without overlap at 1080 by 2400. The
new fixed-lag row showed graph failures, accepted/rejected feedback, applied
correction magnitude, and the stable feedback reason within its panel. The app
started and stopped its foreground session from the UI; filtered AndroidRuntime
and app logcat contained no errors.

The 102.9-second closed session `20260718_074831_f9e20eea` contained 20,525 IMU
records, 102 GNSS clocks, 714 raw measurements, 157 fused fixes, two metadata
records, and exactly one start/end pair. Strict validation returned `valid: true`
with no errors. Replay measured 10,263 accelerometer and 10,262 gyroscope samples
at 100 Hz each with monotonic timestamps, one clock arc, and 102 exact raw callback
epochs. The emulator provided no navigation messages and nonphysical constellation
timing; all epochs were rejected explicitly and the solver truthfully returned
`not_run_no_clean_epochs`.

The live FGO feedback path is instead covered by deterministic JVM execution. It
extracts positive marginal graph covariance, accepts a bounded same-keyframe
correction, rejects stale/missing-covariance/overlarge corrections with stable
reasons, and proves correlated feedback does not shrink EKF covariance. Emulator
GNSS cannot exercise that path because it cannot form solver-ready epochs.

The final default-native 0.9.0 APK has SHA-256
`4afc87abb21c284a25b67ce79fc7f792b3f5f5cad6325a02f1fa9868bc1a87a6` and contains
a 344,120-byte stripped AArch64 `libnavgraph_estimator.so`. Its dependency-free
Android test APK has SHA-256
`d864140bc41ba09651bc641fd5f142876e02ae432f6954f8efb0eea7a8082849`.
ARM instrumentation reported the native library loaded, eight observations solved
in four iterations, `4.01e-9 m` position error, `4.04e-13 m/s` velocity error, 21
live `C++ JNI` WLS updates, zero native failures, 2,000 EKF propagations, four FGO
updates, and one accepted feedback correction. Idle Diagnostics immediately showed
`C++ JNI READY` and still fit within the live-estimator panel.

The headless cold boot used for instrumentation produced one Android System UI ANR
dialog after the tests had passed; choosing Wait restored the system UI. No
NavGraph process exception was present. The earlier visible cold boot completed
the full recording lifecycle without this host-emulator artifact.

The 0.10.0 native-INS verification extends the same arm64 instrumentation through
the high-rate nominal mechanization path. It reported `live_ins_engine=C++ JNI`,
exactly 2,000 native INS propagations, zero native INS failures, 21 native WLS
updates, four FGO updates, and one accepted feedback correction. Independent C++
tests cover gravity cancellation, analytic acceleration/yaw, accelerometer and gyro
bias cancellation, clock propagation, duplicate and mismatched timestamps, and
non-finite input. Host JNI matches the Kotlin full nominal state within `2e-12`.
The verified app APK has SHA-256
`3e67540bf2daf356236adff0b4c622508540e4af5201b17c6d8f1586eff4a1ee` and contains
a 355,808-byte stripped AArch64 `libnavgraph_estimator.so`. Its instrumentation APK
has SHA-256
`fa59ce1c9d946c916d1f225561bb3b86f97f23aafd7966ea51c90a5ca2b117a6`.

Diagnostics now gives WLS/EKF, INS propagation, and fixed-lag FGO separate rows.
The 1080 by 2400 screenshot confirms the added INS engine and fallback counters
are visible without clipping or overlap, with a hint of the following Log section
remaining at the bottom of the viewport.

The final 0.11.0 build moves sparse 17-state covariance propagation and sequential
pseudorange/Doppler Joseph correction into the same portable C++ core. Host JNI
matches Kotlin across the complete nominal state and all 289 covariance entries;
portable tests cover local-frame derivatives, innovation outliers, timestamp and
non-finite failures, and covariance symmetry. Final arm64 instrumentation reported
`live_ekf_engine=C++ JNI`, 2,000 native EKF/INS propagations, 21 native GNSS
corrections, zero WLS/INS/EKF native failures, three asynchronous graph publications,
and one accepted feedback correction. The exact graph-publication count is
scheduler-dependent; successful publication and accepted feedback are the gates.

The verified 0.11.0 app APK has SHA-256
`42f3eee5ab812f931a742486452d3c382995aff5f15c421bd69f7f2181c5f499` and contains
a 392,256-byte stripped AArch64 `libnavgraph_estimator.so`. Its instrumentation APK
has SHA-256
`9ead47e30c823b2bd47bf8f7d98c463146cd2ee8f614b8b45b0f778649167de0`.
The corrected 1080 by 2400 Diagnostics screenshot shows compact WLS and EKF engine
rows together with INS and FGO status, without clipping or overlap.

The final 0.12.0 build adds the portable 11-state fixed-lag graph backend. C++ unit
tests converge in four iterations, reduce a 6.16 m seeded position error to 1.41 m
with a 35 m pseudorange outlier present, return positive marginal sigma, and reject
invalid frames/preintegration. Host JNI matches Kotlin for every keyframe component,
iteration/convergence result, and all eight selected marginal sigmas within `2e-7`.
Final arm64 instrumentation reported `live_fgo_engine=C++ JNI`, 44 native windows,
zero FGO native failures, ten graph publications, and ten accepted feedback
corrections. The publication count is scheduler-dependent; engine, zero fallback,
successful publication, and accepted feedback are the gates.

The verified 0.12.0 app APK has SHA-256
`ab68611e5e3dc040e7f6a67f0e318e4e78c4c3df65e002070ba8e3c187b70026` and contains
a 468,744-byte stripped AArch64 `libnavgraph_estimator.so`. Its instrumentation APK
has SHA-256
`f96fb5c11c1a97d29c2a04841cde421cd0fd6bb8af54a9fca68bdbd25b6e0f48`.
The final 1080 by 2400 Diagnostics screenshot shows native WLS, EKF, INS, and FGO
readiness/counters without overlap or clipping.

The 600-epoch Samsung segment produced the same trajectory metrics as the desktop
reference: WLS horizontal P50/P95 of 8.90/20.43 m and FGO P50/P95 of 8.99/20.20 m.
FGO reduced maximum jump from 40.68 to 37.94 m and second-difference RMS from 11.87
to 9.16 m, while making median error slightly worse. One WLS and three FGO epochs
did not meet the strict convergence threshold. The source mirror has no Doppler,
C/N0, elevation, or IMU, and the UI reports these fields as unavailable.

The five-epoch GSDC 2023 Pixel 7 Pro fixture remains a JVM regression for the rich
derived schema and Doppler Jacobians. It is not used as sustained performance
evidence.

The deterministic GNSS/INS scenario produced horizontal P50 errors of 2.42 m for
WLS, 0.83 m for EKF, and 1.08 m for inertial FGO. P95 was 3.99, 1.35, and 2.93 m,
respectively. The EKF accepted 167 pseudoranges and rejected the one injected
outlier; all 168 Doppler observations were accepted. Final EKF position standard
deviations were 0.47/0.47/1.24 m in ENU. This validates implementation behavior on
synthetic data, not smartphone field accuracy.

The combined 600-second public replay plus 20-second, 100 Hz synthetic GNSS/INS run
completed in 4,472.4 ms on the first cold run after fresh installation and 1,013.2
ms on a subsequent warm run on the one-core ARM emulator. A final reinstalled,
warmed verification run completed in 1,133.8 ms. The loading view remained rendered
and responsive while estimation ran off the UI thread. These are emulator compute
measurements, not phone latency or battery evidence.

Screenshots:

- `images/live.png`
- `images/live-recording.png`
- `images/replay.png`
- `images/replay-on-device.png`
- `images/replay-gnss-ins-top.png`
- `images/replay-gnss-ins-mid.png`
- `images/replay-gnss-ins-bottom.png`
- `images/replay-samsung-loading.png`
- `images/replay-samsung-top.png`
- `images/replay-samsung-bottom.png`
- `images/diagnostics.png`
- `images/diagnostics-live-preprocessor.png`
- `images/diagnostics-broadcast-ephemeris.png`
- `images/live-estimator-v080.png`
- `images/diagnostics-live-estimator-v080.png`
- `images/navgraph_v090_live.png`
- `images/navgraph_v090_recording.png`
- `images/navgraph_v090_diag.png`
- `images/navgraph_v090_native_diag.png`
- `images/navgraph_v010_native_ins_diag.png`
- `images/navgraph_v011_native_ekf_diag.png`
- `images/navgraph_v012_native_fgo_diag.png`
- `images/permission.png`

## Map feature 0.13.0 / code 13 — 2026-11-06

This section applies to branch `codex/feature-osm-trajectory`. Evidence is Android
ARM64 emulator/public replay and deterministic tests, not physical S23+ evidence.
Full feature acceptance remains incomplete for the gaps below.

### Executed verification

- Offline Gradle `testDebugUnitTest`, `assembleDebug` and `lintDebug`: passed;
  36 JVM tests, zero failures. Six focused map tests cover coordinate conversion,
  source separation/deduplication/reset, clock boundaries during duplicate status updates, gaps/invalid samples/time rollback/
  antimeridian splits/staleness, bounded endpoints, and seeking without future or
  live-time samples. Lint retains translation and programmatic-view warnings;
  no build errors were suppressed.
- APK installed and public replay run through the Replay tab. A limited online
  viewing check rendered real OSM streets. Subsequent viewport captures disabled
  Wi-Fi and mobile data and rendered cached streets. No camera-sweep downloads
  or region prefetch were performed.
- Packaged native instrumentation passed on arm64-v8a: C++ JNI WLS/INS/EKF/FGO
  loaded, 2,000 INS and EKF propagations, 21 native GNSS updates, 40 native FGO
  windows, zero native failures and seven accepted feedback corrections in the
  captured run. Native WLS position/velocity errors were 4.01e-9 m / 4.04e-13 m/s.
- The instrumentation additionally ran the bundled public replay and exported
  geographic samples and metrics to the app-private `map-replay-evidence.json`.
  A desktop replay used the identical versioned segment and two-second lag.
  Asset and desktop input bytes match. SHA-256 values are
  `378172102ac5f22984605b91d3329d5612db3a569fbc0c5e1d1db899d2ee081d`
  (device GNSS) and
  `3fe3f086a66f48fc17b408065bbcb67eb32a36e31f70cba3f34d333c37b36201`
  (ground truth). Provenance remains in `datasets/gsdc_2021_s20ultra_segment`.
- All 1,200 Android WLS/FGO geographic samples were compared with desktop ECEF
  conversions: maximum latitude/longitude difference 2.274e-13 degrees. P50/P95
  metric differences are below 2.4e-9 m. See
  [comparison evidence](evidence/map_0_13_0/android_desktop_comparison.json).
- Public replay: 600 epochs, 15,554 accepted observations, WLS/FGO failures 1/3,
  WLS/FGO post-fit rejection totals 15/14 on desktop. Minimum unique satellites
  before post-fit rejection was 12. All 600 epochs in each pipeline retain at
  least 11 under the conservative unique-satellite-count minus rejection-count
  check. These qualifying epochs give WLS P50/P95 8.9042/20.4333 m and FGO
  8.9908/20.2040 m. These are public S20 Ultra regression results.

### Viewports and screenshots

The connected Android Studio ARM64 AVD is `sdk_gphone64_arm64` (Android API 36
family). Its default viewport was 1080 x 2400 at 420 dpi. Overrides tested:

| Configuration | Pixels | Density | Logical size | Font scales |
| --- | --- | --- | --- | --- |
| S23+-representative | 1080 x 2340 | 420 dpi | about 411 x 891 dp | 1.0, 1.3 |
| Smaller supported phone | 1080 x 2160 | 480 dpi | 360 x 720 dp | 1.0, 1.3, 2.0 |

These logical sizes include system-bar areas; the dashboard applies actual
system-bar/cutout insets. Ten replay/top-and-scrolled screenshots and five UI
hierarchies are in [map evidence](evidence/map_0_13_0/). Inspection found a header
system-bar overlap and a slider touch height below 48 dp. The header now applies
insets, and the slider has an explicit 48 dp layout height. Icon controls have
accessible follow/fit/zoom descriptions. Text wraps and the content scrolls at
2.0 font scale; Diagnostics wraps within its tab. Emulator size, density, fonts
and network settings are restored after the capture sequence.

### Remaining acceptance gaps

- Standalone portable C++ and host-JNI parity gates were attempted and failed to
  link: the host linker cannot parse the installed MacOSX27 SDK TBD architectures
  (`arm64e.x1-macos` / `arm64e-macos`). Retrying with Xcode's developer directory
  did not resolve the host mismatch. Packaged Android native parity passed.
- Full live-moving-session/background-history/reset stress checks, all speed/layer/
  follow/fit combinations, and the complete Live/Diagnostics enlarged-font matrix
  are not yet certified. The screenshots establish replay layout observations,
  not a pass for every acceptance interaction.
- An automated local-tile camera harness and a controlled uncached tile-failure
  test remain outstanding. Cached offline rendering was observed; that alone
  does not certify every unavailable-network/error-recovery path.
- There is no physical Samsung S23+ accuracy, power or thermal evidence, and no
  sub-meter claim. Process-death map/session recovery remains out of scope.

Additional interaction evidence: play/pause advanced the recording, seeking selected
299.5 / 599.0 seconds, and background/foreground plus landscape/portrait activity
recreation preserved that timestamp. The restored Replay tab is checked correctly
after fixing an initial tab-highlight mismatch. See `seek_midpoint.png` and
`recreated_midpoint.png` in the evidence directory. UI automation occasionally
reported an idle-state timeout while map frames continued rendering; captured
hierarchies/screenshots and the restored timeline were inspected separately.

Stopped-session log check: exported `navgraph_20261106_154141_a987f467.ndjson`
from the final APK and ran `validation_tools/validate_ndjson.py`: valid, zero
errors; 1,522 IMU, seven GNSS clock, 49 raw GNSS and six fused-location records,
with one session start and one session end. This is an emulator collection check.

## FGO horizon update (2026-11-07; host verification only)

The inertial fixed-lag graph now defaults to a five-second horizon capped at twelve
keyframes, and the live worker solves only the newest window over a six-second
history. This section records host evidence only; it does not claim Android playback
or device verification.

Host checks from `fgo_android_app`:

- `:app:testDebugUnitTest` — 40 tests, 0 failures, 0 skipped.
- `:app:nativeHostParityTest` — the new six-keyframe
  (`jniMatchesKotlinAtTheFiveSecondWindowSize`) Kotlin/JNI parity case passed for
  every state component and selected marginal.
- `:app:assembleDebug :app:lintDebug` — build and lint passed; the arm64 CMake debug
  library rebuilt. Lint reports only pre-existing `SetTextI18n` warnings in UI files.
- `./tools/run_cpp_tests.sh` — all four portable C++ host tests passed.
- `python3 -m unittest discover -s validation_tools -p "test_*.py"` — 12 tests passed.
- The deterministic synthetic FGO changed from P50/P95 1.08/2.93 m at a two-second
  horizon to 0.96/2.03 m at five seconds, with zero solver failures; second-difference
  RMS changed from 3.84 m to 2.32 m.

Not executed for this change:

- Android Studio AVD playback of public smartphone data with the new horizon.
- UI sizing at S23+-representative and smaller phone viewports, including enlarged
  system fonts.
- Physical S23+ rate, latency, power, thermal, or field-accuracy profiling.

The 600-epoch GSDC replay still uses the GNSS-only two-second FGO so its desktop and
Android metrics remain comparable. Until the AVD checks above are run and recorded,
the live horizon change is host-verified only.
