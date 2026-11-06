package com.example.navgraph.live

enum class ImuSensorType {
    ACCELEROMETER,
    GYROSCOPE,
}

sealed interface NavigationEvent {
    val elapsedRealtimeNanos: Long
}

data class RawGnssClockSample(
    override val elapsedRealtimeNanos: Long,
    val callbackElapsedRealtimeNanos: Long,
    val timeNanos: Long,
    val hardwareClockDiscontinuityCount: Int,
    val fullBiasNanos: Long?,
    val biasNanos: Double?,
    val biasUncertaintyNanos: Double?,
    val driftNanosPerSecond: Double?,
    val driftUncertaintyNanosPerSecond: Double?,
) : NavigationEvent

data class RawGnssObservationSample(
    val constellation: Int,
    val svid: Int,
    val state: Int,
    val timeOffsetNanos: Double,
    val receivedSvTimeNanos: Long,
    val receivedSvTimeUncertaintyNanos: Long,
    val cn0DbHz: Double,
    val pseudorangeRateMetersPerSecond: Double,
    val pseudorangeRateUncertaintyMetersPerSecond: Double,
    val carrierFrequencyHz: Double?,
    val codeType: String,
    val fullInterSignalBiasNanos: Double?,
    val satelliteInterSignalBiasNanos: Double?,
)

data class RawGnssEpochSample(
    override val elapsedRealtimeNanos: Long,
    val clock: RawGnssClockSample,
    val observations: List<RawGnssObservationSample>,
) : NavigationEvent

data class RawGnssNavigationMessageSample(
    override val elapsedRealtimeNanos: Long,
    val svid: Int,
    val messageType: Int,
    val messageId: Int,
    val submessageId: Int,
    val status: Int,
    val data: ByteArray,
) : NavigationEvent

data class RawImuSample(
    override val elapsedRealtimeNanos: Long,
    val callbackElapsedRealtimeNanos: Long,
    val sensorType: ImuSensorType,
    val values: DoubleArray,
    val accuracy: Int,
) : NavigationEvent {
    init {
        require(values.size == 3)
    }
}

data class FusedLocationSample(
    override val elapsedRealtimeNanos: Long,
    val callbackElapsedRealtimeNanos: Long,
    val provider: String,
    val latitudeDegrees: Double,
    val longitudeDegrees: Double,
    val altitudeMeters: Double?,
    val horizontalAccuracyMeters: Double,
) : NavigationEvent

fun interface NavigationEventSink {
    /** Returns false when the bounded downstream queue cannot accept the event. */
    fun offer(event: NavigationEvent): Boolean
}
