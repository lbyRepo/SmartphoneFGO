# Samsung S23+ GNSS/INS/FGO Prototype

NavGraph is an Android research prototype for raw GNSS and IMU recording, live
INS/EKF navigation, asynchronous fixed-lag FGO, and deterministic public-data
replay. The active Android project is **`fgo_android_app`**. Emulator and public
replay results do not establish physical S23+ or sub-meter field performance.

## Repository structure

- `fgo_android_app/`: active Android app and portable C++/JNI estimator core.
- `fgo_android_app/docs/`: canonical technical contracts and verification reports.
- `validation_tools/`: independent desktop reference, replay and log validation.
- `datasets/`: versioned public fixtures with provenance and licensing.
- `assets/`: generated PNG images and screenshots.
- `android_app/`, `cpp_core/`, `python_tools/`, `docs/`: earlier prototype material;
  use the active project and documentation linked above for current development.

## Prerequisites

Install these tools locally:

- A JDK 17 or newer compatible with Gradle 8.13. Set `JAVA_HOME` to its installation.
- Gradle **8.13**, with `gradle` on `PATH`.
- Android Studio and Android SDK platform **36**, SDK platform-tools, NDK
  **29.0.14206865**, and SDK CMake **3.22.1**. Set `ANDROID_HOME` to your SDK
  installation (its location is shown in Android Studio's SDK Manager).
- Python **3.10+** for desktop validation; NumPy for replay/benchmark calculations.

No particular clone location, username or Android Studio installation path is
required. SDK and JDK installations live outside the repository, so their paths
are supplied by your environment. You can alternatively configure the SDK using
an untracked `fgo_android_app/local.properties` containing `sdk.dir=...` with your
own SDK path. Configure Android Studio's Gradle JDK to use your installed JDK.

The shell examples below run from the **repository root**, using a POSIX shell
(macOS/Linux, or an appropriate Windows shell). On Windows, configure environment
variables using your shell's syntax and use `.venv\Scripts\python.exe` for Python.

## Build and install

```bash
# Use the SDK installed on your machine.
export PATH="$JAVA_HOME/bin:$ANDROID_HOME/platform-tools:$PATH"

gradle -p fgo_android_app :app:testDebugUnitTest :app:assembleDebug :app:lintDebug
adb install -r fgo_android_app/app/build/outputs/apk/debug/app-debug.apk
```

The first build needs internet access to resolve dependencies. Add `--offline`
only after the required dependencies have been cached. These instructions use an
installed Gradle 8.13; the repository-root wrapper belongs to the earlier prototype.

APK: `fgo_android_app/app/build/outputs/apk/debug/app-debug.apk`.
The default APK includes native libraries for **arm64-v8a**; use an ARM64 AVD or
compatible physical device. A deliberate Kotlin-only fallback build can use
`-Pnavgraph.enableNative=false` when the native toolchain is unavailable.

## Record and validate a session

1. Open **NavGraph** and grant location and notification permissions.
2. Tap **Start recording**. Test outdoors when assessing GNSS capability.
3. Tap **Stop and save** before exporting the session.

```bash
adb pull /sdcard/Android/data/com.example.navgraph/files/Documents/logs ./device_logs
python3 validation_tools/validate_ndjson.py device_logs/SESSION.ndjson
```

Replace `SESSION.ndjson` with the exported filename. The `/sdcard/...` path is an
Android-device path, independent of the directory where this repository is cloned.

For desktop replay, create an environment inside the clone:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install numpy
.venv/bin/python validation_tools/navgraph_replay.py device_logs/SESSION.ndjson \
  --output validation_results/SESSION
```

## Further documentation

- [Android app and native checks](fgo_android_app/README.md)
- [Desktop benchmark and replay](validation_tools/README.md)
- [Log schema](fgo_android_app/docs/log_schema.md)
- [Estimator design](fgo_android_app/docs/estimator_design.md)
- [Testing plan](fgo_android_app/docs/testing_plan.md)
- [Emulator verification and limitations](fgo_android_app/docs/emulator_verification.md)
- [Repository agent guide](AGENTS.md)
