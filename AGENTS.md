# Repository agent guide

The active Android GNSS/INS/FGO prototype is rooted at `fgo_android_app`.

Before changing that project, read and follow
[`fgo_android_app/AGENTS.md`](fgo_android_app/AGENTS.md). It defines the time,
coordinate-frame, GNSS, schema, testing, and accuracy-claim rules for this
workspace.

Repository-level rules:

- Save all future generated PNG images under the repository-root `assets/`
  directory, using task-specific subfolders when useful. Do not write generated
  PNGs directly into the repository root. Link to this location from verification
  reports and other documentation.
- Treat `fgo_android_app/docs` as the canonical technical documentation.
- Keep `validation_tools` as an independent desktop reference and log validator.
- Keep public fixtures under `datasets` or bundled read-only Android assets; record
  provenance and do not tune against test ground truth.
- Do not edit generated Gradle outputs, validation results, or local IDE metadata.
- Do not claim physical Samsung S23+ or sub-meter performance from emulator,
  synthetic, or short public-fixture evidence.

## Android playback and UI validation

- Every new feature must be verified by replaying public smartphone data inside
  the built Android app on an Android Studio Android Virtual Device (AVD).
  Desktop replay alone does not satisfy this requirement. Use the same versioned
  dataset and estimator configuration for Android and desktop comparisons.
- Validate UI sizing during playback at an S23+-representative viewport and a
  smaller supported phone viewport, including enlarged system fonts. Check text,
  controls, plots, scrolling, and system-bar insets for readability and reachability.
- Follow the measurable playback and layout acceptance criteria in
  `fgo_android_app/docs/testing_plan.md`. Record screenshots, configuration,
  results, and unresolved failures in `fgo_android_app/docs/emulator_verification.md`.
- If the emulator or suitable replay data is unavailable, report validation as
  incomplete. Do not claim UI or playback verification from a build alone.

## Git and commit proposals

- Do not create commits. Agents may stage task-related changes and propose a
  commit message and description for the user to review. Do not push or open a
  pull request unless explicitly requested.
- Always include a proposed commit title and description when handing off file
  changes, whether staged or unstaged. Report staging status accurately and never
  describe a proposed commit as an existing commit.
- Use Conventional Commits syntax: `type(scope): imperative summary`. The scope
  is optional. Use `feat` for new functionality, `fix` for bug fixes, `chore` for
  maintenance, `docs` for documentation, `test` for tests, `refactor` for behavior-
  preserving restructuring, and `build` or `ci` for their respective tooling.
  Examples: `feat(logging): record GNSS clock discontinuities`,
  `fix(replay): preserve monotonic timestamp ordering`, and
  `chore(repo): initialize Git and document commit conventions`.
- Keep the title concise and specific. In the proposed description, explain what
  changed and why, summarize verification actually performed, and disclose any
  remaining limitations. Do not claim tests passed unless they were run.
- Mark breaking changes with `!` after the type/scope and explain the migration
  in a `BREAKING CHANGE:` paragraph.
- Inspect the diff before staging. Stage only files relevant to the task; exclude
  credentials, local configuration, generated builds, and unrelated user changes.
