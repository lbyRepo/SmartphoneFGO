package com.example.navgraph.live

import java.io.Closeable
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.TimeUnit
import java.util.concurrent.atomic.AtomicBoolean
import java.util.concurrent.atomic.AtomicLong

data class NavigationBusHealth(
    val acceptedEvents: Long,
    val processedEvents: Long,
    val droppedEvents: Long,
    val consumerFailures: Long,
    val queuedEvents: Int,
)

class BoundedNavigationEventBus(
    capacity: Int,
    private val consumer: (NavigationEvent) -> Unit,
) : NavigationEventSink, Closeable {
    private val queue = ArrayBlockingQueue<NavigationEvent>(capacity)
    private val running = AtomicBoolean(false)
    private val accepted = AtomicLong()
    private val processed = AtomicLong()
    private val dropped = AtomicLong()
    private val failures = AtomicLong()
    private val worker = Thread(::runWorker, "navgraph-live-estimator")

    init {
        require(capacity > 0)
    }

    fun start() {
        if (running.compareAndSet(false, true)) worker.start()
    }

    override fun offer(event: NavigationEvent): Boolean {
        if (!running.get()) {
            dropped.incrementAndGet()
            return false
        }
        val offered = queue.offer(event)
        if (offered) accepted.incrementAndGet() else dropped.incrementAndGet()
        return offered
    }

    fun health(): NavigationBusHealth = NavigationBusHealth(
        acceptedEvents = accepted.get(),
        processedEvents = processed.get(),
        droppedEvents = dropped.get(),
        consumerFailures = failures.get(),
        queuedEvents = queue.size,
    )

    override fun close() {
        if (!running.getAndSet(false)) return
        worker.interrupt()
        worker.join(2_000L)
    }

    private fun runWorker() {
        while (running.get() || queue.isNotEmpty()) {
            val event = try {
                queue.poll(100L, TimeUnit.MILLISECONDS)
            } catch (_: InterruptedException) {
                null
            } ?: continue
            try {
                consumer(event)
            } catch (_: RuntimeException) {
                failures.incrementAndGet()
            } finally {
                processed.incrementAndGet()
            }
        }
    }
}
