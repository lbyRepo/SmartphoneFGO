# Samsung S23+ GNSS/INS/FGO Prototype

Initial repository architecture for a Samsung Galaxy S23+ GNSS/INS/FGO prototype. The implemented code currently covers the Android diagnostic logger and replay schema validation; estimator math is intentionally interface-only until the logging and replay milestone is complete.

## Repository Structure

- `android_app/`: Kotlin Android app for permissions, foreground logging, raw GNSS, IMU, and fused-location collection.
- `cpp_core/`: C++ estimator interfaces and future GNSS/INS/FGO core boundaries.
- `python_tools/`: replay, validation, plotting, and offline analysis tools.
- `python_tools/gnss/`: GNSS preprocessing and coordinate transforms for replayed logs.
- `tests/`: fixtures and future unit/integration/replay tests.
- `docs/`: schema, ownership, roadmap, test strategy, and interface documentation.

## What It Logs

- Android Raw GNSS measurements via `GnssMeasurementsEvent.Callback`
- GNSS clock fields for each raw measurement epoch
- Accelerometer and gyroscope samples at a requested 100 Hz period
- Android platform fused location provider output for baseline comparison
- Sensor metadata needed to confirm sampling rates and FIFO behavior

Logs are newline-delimited JSON files following [docs/log_schema.md](docs/log_schema.md).

## Build

This workspace is configured for the local Android SDK at `/Users/leeboonyao/Library/Android/sdk` and Android Studio's bundled JBR.

```bash
/Users/leeboonyao/.gradle/wrapper/dists/gradle-8.13-bin/5xuhj0ry160q40clulazy9h7d/gradle-8.13/bin/gradle :android_app:app:assembleDebug --offline
```

Debug APK:

```text
android_app/app/build/outputs/apk/debug/app-debug.apk
```

## Device Workflow

Install:

```bash
/Users/leeboonyao/Library/Android/sdk/platform-tools/adb install -r android_app/app/build/outputs/apk/debug/app-debug.apk
```

On the phone:

1. Open **S23 GNSS Logger**.
2. Grant location and notification permissions.
3. Tap **Start logging**.
4. Test outdoors with clear sky view when checking GNSS capability.
5. Tap **Stop logging**.

Logs are written under the app external files directory:

```text
/sdcard/Android/data/com.example.s23logger/files/Documents/logs/
```

Pull logs:

```bash
/Users/leeboonyao/Library/Android/sdk/platform-tools/adb pull /sdcard/Android/data/com.example.s23logger/files/Documents/logs ./device_logs
```

Validate a log:

```bash
python3 python_tools/replay/parse_log.py --strict device_logs/<log-file>.ndjson
```

## Architecture Docs

- [AGENTS.md](AGENTS.md): task routing and edit boundaries for future Codex agents.
- [docs/module_ownership.md](docs/module_ownership.md): module ownership map.
- [docs/estimator_interfaces.md](docs/estimator_interfaces.md): interface contracts before estimator implementation.
- [docs/gnss_preprocessing.md](docs/gnss_preprocessing.md): raw GNSS replay to observation epoch conversion.
- [docs/replay_evaluation.md](docs/replay_evaluation.md): single-command replay, metrics, and plots.
- [docs/test_strategy.md](docs/test_strategy.md): staged test strategy.
- [docs/implementation_roadmap.md](docs/implementation_roadmap.md): milestones and first checklist.

## Notes

- The app requests full-tracking raw GNSS measurements on Android 12+.
- `HIGH_SAMPLING_RATE_SENSORS` is declared so IMU sampling is not capped at 200 Hz by that permission gate.
- `wall_time_millis` is for correlation only. Replay/fusion code should use monotonic timestamps and GNSS clock-domain fields described in the schema.
