package com.example.navgraph.estimator

/**
 * Shared fixed-lag FGO policy.
 *
 * The inertial graph previously used a two-second window. On the deterministic
 * GNSS/INS scenario that window is information-starved: lengthening it to five
 * seconds reduces horizontal P95 by roughly a third and second-difference RMS by
 * about 40 percent, while the 1 Hz window solve remains well below one epoch of
 * compute. [MAX_KEYFRAMES] bounds the dense Gauss-Newton matrix at higher GNSS
 * rates; a 5 s window at 1 Hz stays at six 11-state keyframes (66 by 66).
 */
object FgoConfig {
    /** Default backward smoothing horizon for the inertial graph. */
    const val DEFAULT_LAG_SECONDS = 5.0

    /** Hard bound on keyframes in one dense Gauss-Newton window. */
    const val MAX_KEYFRAMES = 12

    /** A window must contain at least the current keyframe and one history keyframe. */
    const val MIN_KEYFRAMES = 2
}
