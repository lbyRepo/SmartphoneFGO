package com.example.navgraph.live

import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class BoundedNavigationEventBusTest {
    @Test
    fun drainsAcceptedEventsAndRejectsAfterClose() {
        val received = mutableListOf<Long>()
        val bus = BoundedNavigationEventBus(8) { event ->
            synchronized(received) { received += event.elapsedRealtimeNanos }
        }
        bus.start()
        repeat(5) { index -> assertTrue(bus.offer(imu(index.toLong()))) }
        bus.close()

        assertEquals(listOf(0L, 1L, 2L, 3L, 4L), synchronized(received) { received.toList() })
        assertEquals(5L, bus.health().processedEvents)
        assertEquals(0L, bus.health().droppedEvents)
        assertFalse(bus.offer(imu(6L)))
        assertEquals(1L, bus.health().droppedEvents)
    }

    private fun imu(timestamp: Long) = RawImuSample(
        elapsedRealtimeNanos = timestamp,
        callbackElapsedRealtimeNanos = timestamp + 1,
        sensorType = ImuSensorType.ACCELEROMETER,
        values = doubleArrayOf(0.0, 0.0, 9.81),
        accuracy = 3,
    )
}
