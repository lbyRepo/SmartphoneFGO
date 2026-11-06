package com.example.navgraph.ui

import android.content.Context
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import android.widget.*
import com.example.navgraph.map.*
import org.maplibre.android.MapLibre
import org.maplibre.android.maps.*
import org.maplibre.android.camera.CameraUpdateFactory
import org.maplibre.android.geometry.LatLng
import org.maplibre.android.geometry.LatLngBounds
import org.maplibre.android.style.sources.GeoJsonSource
import org.maplibre.android.style.layers.LineLayer
import org.maplibre.android.style.layers.CircleLayer
import org.maplibre.android.style.layers.PropertyFactory.*
import org.json.JSONArray
import org.json.JSONObject

/** Rendering consumes display samples only; never feeds navigation computations. */
class StreetMapPanel(context: Context, private val replay: Boolean, private val saved: Bundle? = null) : LinearLayout(context) {
    private val mapView: MapView
    private var map: MapLibreMap? = null
    private val status = TextView(context)
    private val position = TextView(context)
    private var samples = emptyList<GeographicSample>()
    private val registeredSources = linkedSetOf<String>()
    private val timeline = TextView(context)
    private var mapAvailable = true
    private val hidden = mutableSetOf<String>()
    private var follow = true
    private var cursor = Long.MAX_VALUE
    private var playing = saved?.getBoolean("playing", false) ?: false
    private var speed = 1L
    private val preparation = java.util.concurrent.Executors.newSingleThreadExecutor()
    private var generation = 0L
    private var destroyed = false
    private val handler = Handler(Looper.getMainLooper())
    private val layers = LinearLayout(context).apply { orientation = VERTICAL }
    private val seek = SeekBar(context)
    private val tick = object : Runnable {
        override fun run() {
            if (!isShown) { handler.postDelayed(this, 250); return }
            if (replay && playing) {
                cursor = minOf(cursor + 250 * speed, samples.maxOfOrNull { it.timestamp } ?: cursor)
                if (cursor >= (samples.maxOfOrNull { it.timestamp } ?: cursor)) playing = false
                render()
            } else if (!replay) setSamples(LiveMapSession.history.snapshot().values.flatten())
            handler.postDelayed(this, if (replay) 250 else 1000)
        }
    }
    init {
        orientation = VERTICAL
        MapLibre.getInstance(context)
        mapView = MapView(context)
        MapNetworking.configure(context)
        follow = saved?.getBoolean("follow", true) ?: true
        cursor = saved?.getLong("cursor", Long.MAX_VALUE) ?: Long.MAX_VALUE
        speed = saved?.getLong("speed", 1L) ?: 1L
        hidden.addAll(saved?.getStringArrayList("hidden") ?: emptyList())
        mapView.onCreate(saved?.getBundle("map"))
        addView(status)
        addView(mapView, LayoutParams(LayoutParams.MATCH_PARENT, (300 * resources.displayMetrics.density).toInt()))
        addView(TextView(context).apply {
            text = "© OpenStreetMap contributors"; textSize = 14f
            setOnClickListener { context.startActivity(android.content.Intent(android.content.Intent.ACTION_VIEW,
                android.net.Uri.parse("https://www.openstreetmap.org/copyright"))) }
            minHeight = dp(48)
        })
        addView(position)
        val controls = LinearLayout(context)
        fun control(label: String, symbol: String = label, action: () -> Unit) { controls.addView(Button(context).apply {
            text = symbol; contentDescription = label; minWidth = dp(48); minHeight = dp(48)
            setOnClickListener { action() }
        }, LayoutParams(0, LayoutParams.WRAP_CONTENT, 1f)) }
        control("Follow position", "⊙") { follow = true; render() }
        control("Fit trajectory", "⇱") { follow = false; fit() }
        control("Zoom in", "+") { map?.animateCamera(CameraUpdateFactory.zoomIn()) }
        control("Zoom out", "−") { map?.animateCamera(CameraUpdateFactory.zoomOut()) }
        addView(controls)
        addView(layers)
        if (replay) {
            addView(TextView(context).apply { text = "Recorded-result playback • FGO selected"; textSize = 14f })
            val playback = LinearLayout(context)
            playback.addView(Button(context).apply { text = "Play / pause"; minHeight = dp(48)
                setOnClickListener {
                    if (cursor >= (samples.maxOfOrNull { it.timestamp } ?: cursor)) cursor = samples.minOfOrNull { it.timestamp } ?: cursor
                    playing = !playing
                } }, LayoutParams(0, LayoutParams.WRAP_CONTENT, 1f))
            playback.addView(Spinner(context).apply {
                adapter = ArrayAdapter(context, android.R.layout.simple_spinner_dropdown_item, listOf("1x", "5x", "10x"))
                setSelection(listOf(1L, 5L, 10L).indexOf(speed).coerceAtLeast(0))
                minimumHeight = dp(48)
                onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
                    override fun onNothingSelected(parent: AdapterView<*>?) = Unit
                    override fun onItemSelected(parent: AdapterView<*>?, view: android.view.View?, index: Int, id: Long) {
                        speed = listOf(1L, 5L, 10L)[index]
                    }
                }
            })
            addView(playback)
            status.text = "Map loading"
            seek.max = 10000
            seek.minimumHeight = dp(48)
            seek.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
                override fun onStartTrackingTouch(bar: SeekBar?) { playing = false }
                override fun onStopTrackingTouch(bar: SeekBar?) = Unit
                override fun onProgressChanged(bar: SeekBar?, progress: Int, fromUser: Boolean) {
                    if (fromUser && samples.isNotEmpty()) {
                        val start = samples.minOf { it.timestamp }; val end = samples.maxOf { it.timestamp }
                        cursor = start + (end - start) * progress / 10000; render()
                    }
                }
            })
            addView(timeline)
            addView(seek, LayoutParams(LayoutParams.MATCH_PARENT, dp(48)))
        }
        mapView.addOnDidFailLoadingMapListener { status.text = "Map unavailable • coordinates and tracks remain available" }
        mapView.getMapAsync { ready ->
            map = ready
            ready.prefetchZoomDelta = 0
            ready.uiSettings.isRotateGesturesEnabled = false
            ready.uiSettings.isTiltGesturesEnabled = false
            ready.addOnCameraMoveStartedListener { reason ->
                if (reason == MapLibreMap.OnCameraMoveStartedListener.REASON_API_GESTURE) follow = false
            }
            mapView.addOnTileActionListener { operation, _, _, _, _, _, _ ->
                when (operation) {
                    org.maplibre.android.tile.TileOperation.Error -> {
                        mapAvailable = false
                        status.text = "Map unavailable • coordinates and tracks remain available"
                    }
                    org.maplibre.android.tile.TileOperation.LoadFromNetwork,
                    org.maplibre.android.tile.TileOperation.LoadFromCache -> {
                        mapAvailable = true; status.text = "Street map"
                    }
                    else -> Unit
                }
            }
            ready.setStyle(Style.Builder().fromUri("asset://street_map.json")) { render() }
        }
    }
    fun setSamples(next: List<GeographicSample>) {
        val sourcesChanged = samples.map { it.source }.toSet() != next.map { it.source }.toSet()
        samples = next
        if (replay && cursor == Long.MAX_VALUE && next.isNotEmpty()) cursor = next.minOf { it.timestamp }
        if (sourcesChanged) {
            layers.removeAllViews()
            next.map { it.source }.distinct().forEach { source ->
                layers.addView(CheckBox(context).apply {
                    text = source; isChecked = source !in hidden; minHeight = dp(48); textSize = 14f
                    setOnCheckedChangeListener { _, checked -> if (checked) hidden.remove(source) else hidden.add(source); render() }
                })
            }
        }
        render()
    }
    private fun visible() = samples.filter { it.source !in hidden && (!replay || it.timestamp <= cursor) }
    private fun render() {
        if (destroyed) return
        val shown = visible()
        val latest = shown.filter { it.usable }.groupBy { it.source }.mapValues { it.value.last() }
        position.text = if (latest.isEmpty()) "Waiting for position" else latest.values.joinToString("\n") { p ->
            val age = if (replay) (cursor - p.timestamp) / 1000.0 else (SystemClock.elapsedRealtimeNanos() - p.timestamp) / 1e9
            val uncertainty = p.uncertaintyMeters?.takeIf { it.isFinite() }?.let {
                " • ${if (p.source == "Android fused") "Android accuracy" else "estimator sigma"} %.1f m".format(it)
            }.orEmpty()
            "%s %.6f, %.6f • %.1f s%s%s".format(p.source, p.latitudeDegrees, p.longitudeDegrees, age,
                if (age > 5) " • STALE" else "", uncertainty)
        }
        position.textSize = 14f
        val style = map?.style ?: return
        registeredSources.addAll(samples.map { it.source })
        val known = registeredSources.toList()
        val revision = ++generation
        preparation.execute {
        known.forEachIndexed { index, source ->
            val id = "track-$index"
            val track = shown.filter { it.source == source }
            val features = JSONArray()
            geographicSegments(track).filter { it.size >= 2 }.forEach { segment ->
                val coordinates = JSONArray()
                segment.forEach { coordinates.put(JSONArray(listOf(it.longitudeDegrees, it.latitudeDegrees))) }
                features.put(JSONObject().put("type", "Feature").put("properties", JSONObject())
                    .put("geometry", JSONObject().put("type", "LineString").put("coordinates", coordinates)))
            }
            val json = JSONObject().put("type", "FeatureCollection").put("features", features).toString()
            val color = when (source) { "Android fused", "WLS" -> "#2563eb"; "Reference" -> "#e58b22"; else -> "#008d86" }
            val last = track.lastOrNull()?.takeIf { it.usable }
            val marker = if (last == null) "{\"type\":\"FeatureCollection\",\"features\":[]}" else
                "{\"type\":\"Point\",\"coordinates\":[${last.longitudeDegrees},${last.latitudeDegrees}]}"
            handler.post {
            if (destroyed || revision != generation) return@post
            val existing = style.getSourceAs<GeoJsonSource>(id)
            if (existing != null) existing.setGeoJson(json) else {
                style.addSource(GeoJsonSource(id, json))
                style.addLayer(LineLayer("line-$id", id).withProperties(lineColor(color), lineWidth(3f)))
                if (source == "Android fused") style.getLayerAs<LineLayer>("line-$id")?.setProperties(lineDasharray(arrayOf(2f, 2f)))
            }
            val markerId = "point-$id"
            val pointSource = style.getSourceAs<GeoJsonSource>(markerId)
            if (pointSource != null) pointSource.setGeoJson(marker) else {
                style.addSource(GeoJsonSource(markerId, marker))
                style.addLayer(CircleLayer("circle-$id", markerId).withProperties(circleColor(color), circleRadius(6f)))
            }
        }
            }
        }
        if (follow) (latest["FGO"] ?: latest.values.firstOrNull { it.source != "Android fused" } ?: latest.values.firstOrNull())?.let {
            map?.moveCamera(CameraUpdateFactory.newLatLngZoom(LatLng(it.latitudeDegrees, it.longitudeDegrees), (map?.cameraPosition?.zoom ?: 0.0).let { zoom -> if (zoom < 1.0) 15.0 else zoom }))
        }
        if (replay && samples.isNotEmpty()) {
            val start = samples.minOf { it.timestamp }; val span = samples.maxOf { it.timestamp } - start
            timeline.text = "%.1f / %.1f seconds".format((cursor - start) / 1000.0, span / 1000.0)
            timeline.textSize = 14f
            seek.progress = if (span == 0L) 0 else ((cursor - start) * 10000 / span).toInt()
        }
    }
    private fun fit() {
        val points = visible().filter { it.usable }.map { LatLng(it.latitudeDegrees, it.longitudeDegrees) }
        if (points.size >= 2) map?.moveCamera(CameraUpdateFactory.newLatLngBounds(LatLngBounds.Builder().includes(points).build(), dp(30)))
    }
    fun save(): Bundle = Bundle().apply {
        putBoolean("follow", follow); putBoolean("playing", playing); putLong("cursor", cursor); putLong("speed", speed)
        putStringArrayList("hidden", ArrayList(hidden))
        putBundle("map", Bundle().also { mapView.onSaveInstanceState(it) })
    }
    fun start() { mapView.onStart(); mapView.onResume(); handler.removeCallbacks(tick); handler.post(tick) }
    fun stop() { handler.removeCallbacks(tick); mapView.onPause(); mapView.onStop() }
    fun destroy() { destroyed = true; generation++; preparation.shutdownNow(); handler.removeCallbacks(tick); mapView.onDestroy() }
    fun lowMemory() = mapView.onLowMemory()
    private fun dp(value: Int) = (value * resources.displayMetrics.density).toInt()
}

private object MapNetworking {
    private var configured = false
    @Synchronized fun configure(context: Context) {
        if (configured) return
        val client = okhttp3.OkHttpClient.Builder()
            .cache(okhttp3.Cache(java.io.File(context.cacheDir, "street-tiles"), 64L * 1024 * 1024))
            .addInterceptor { chain ->
                chain.proceed(chain.request().newBuilder().header("User-Agent",
                    "NavGraph/${com.example.navgraph.BuildConfig.VERSION_NAME} (Android GNSS research prototype)").build())
            }.build()
        org.maplibre.android.module.http.HttpRequestUtil.setOkHttpClient(client)
        configured = true
    }
}
