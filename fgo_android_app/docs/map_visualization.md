# Geographic map display (0.13.0)

Live and Replay use MapLibre Native Android 13.7.1 with a bundled raster style.
The renderer consumes navigation outputs; map interaction never feeds the estimator.
The application version is 0.13.0 / 13. Log metadata uses BuildConfig.VERSION_NAME.

## Data and time

`ReplayResult.geographicTracks` adds geographic WLS, FGO and reference samples to
the existing ENU trajectories and metrics. Conversion uses each original WGS84
ECEF position. A nonconverged estimate remains present with `valid = false`.
Reference data is used only for display and evaluation.

`GeographicSample` names its source, latitude and longitude in degrees, time base,
optional uncertainty in meters, validity and explicit segment boundary.
Live samples use monotonic nanoseconds; public replay uses dataset milliseconds.
The renderer writes GeoJSON coordinates in longitude, latitude order.

Live history is owned by `LiveMapSession`, updated by LoggingService at its status
cadence even when no activity receives broadcasts. Each estimator source and the
Android fused baseline have separate histories. Android accuracy and estimator
sigma have separate labels. History survives tab changes, activity recreation and
session stop, and resets when the next recording starts. Process-death recovery
and historical session browsing are outside this version.

Display histories deduplicate timestamps per source and cap each source at 20,000
points. Progressive decimation preserves retained segment endpoints. If segment
boundaries alone exhaust the cap, oldest display data is retired and the new
first point starts a segment. Original logs are never simplified. Lines split at
invalid samples, source/time-base changes, non-increasing time, explicit clock
boundaries, gaps longer than five seconds and antimeridian crossings.

## Interaction and lifecycle

Live is map-first with expandable diagnostics. Replay computation completes before
recorded-result playback starts. Play/pause, seek and 1x/5x/10x controls reveal
only geographic samples at or before the selected dataset timestamp. Seeking
backward also removes future markers and lines. FGO is the default follow target;
WLS and reference comparisons are enabled and individually hideable.

The map is north-up. Manual camera gestures disable follow; the follow control
restores it. Fit uses currently revealed, visible tracks. Coordinates remain
available when tiles fail. No marker is fabricated while waiting for a position.
Live samples older than five seconds are labelled STALE.

Map lifecycle calls follow activity start/stop/destroy/low-memory callbacks.
Camera, layer visibility, follow and playback state are saved for recreation;
completed replay results remain in process memory. GeoJSON preparation runs on a
worker; generation checks discard obsolete results after seeking or destruction.
System-bar and cutout insets are applied to the dashboard.

## Tiles

The bundled style requests `https://tile.openstreetmap.org/{z}/{x}/{y}.png`.
Visible attribution links to OpenStreetMap copyright information. An application
User-Agent identifies NavGraph and its build version. The OkHttp client has a
64 MiB HTTP-aware disk cache; MapLibre also manages its normal resource cache.
Prefetch zoom delta is zero. There is no offline-region download or bulk-prefetch
feature. Follow the [OSM tile policy](https://operations.osmfoundation.org/policies/tiles/).

See `emulator_verification.md` for executed checks and remaining acceptance gaps.
